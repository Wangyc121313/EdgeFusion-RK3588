#!/usr/bin/env python3
"""Train lightweight MLP fusion head for distance correction.

Architecture: Input(9) → FC(32) → ReLU → FC(16) → ReLU → FC(1)
Output: predicted distance residual or direct distance.
Loss: Huber (SmoothL1) for robustness to outliers.

Saves:
  - model checkpoint (.pt)
  - C++ header with hardcoded weights (for zero-dependency inference on RK3588)

Usage:
  python3 tools/calib/train_fusion_mlp.py \\
      --data /tmp/fusion_training_data.csv \\
      --out-dir /tmp/fusion_mlp_model
"""

import argparse
import csv
import math
import os
import random
import sys
from typing import List, Tuple

import torch
import torch.nn as nn
import torch.optim as optim
from torch.utils.data import DataLoader, TensorDataset


# ---------------------------------------------------------------------------
# Model
# ---------------------------------------------------------------------------

class FusionMLP(nn.Module):
    """Lightweight MLP: 9 → 32 → 16 → 1"""

    def __init__(self, input_dim: int = 9, hidden1: int = 32, hidden2: int = 16):
        super().__init__()
        self.net = nn.Sequential(
            nn.Linear(input_dim, hidden1),
            nn.ReLU(),
            nn.Linear(hidden1, hidden2),
            nn.ReLU(),
            nn.Linear(hidden2, 1),
        )
        self._input_dim = input_dim

    @property
    def input_dim(self) -> int:
        return self._input_dim

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        return self.net(x)


# ---------------------------------------------------------------------------
# Data loading
# ---------------------------------------------------------------------------

FEATURE_NAMES = [
    "center_x_norm", "center_y_norm", "box_w_norm", "box_h_norm",
    "box_area_ratio", "confidence", "raw_distance_m",
    "candidate_points", "cluster_score",
]


def load_csv(csv_path: str,
             val_ratio: float = 0.15) -> Tuple[TensorDataset, TensorDataset, Tuple[torch.Tensor, torch.Tensor]]:
    """Load CSV, normalize features, split train/val. Returns (train_ds, val_ds, (mean, std))."""
    rows = []
    with open(csv_path, "r", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        for row in reader:
            try:
                feats = [float(row[n]) for n in FEATURE_NAMES]
                target = float(row["target_distance_m"])
                rows.append((feats, target))
            except (KeyError, ValueError):
                continue

    if len(rows) < 10:
        raise ValueError(f"only {len(rows)} samples — need more data")

    X = torch.tensor([r[0] for r in rows], dtype=torch.float32)
    y = torch.tensor([r[1] for r in rows], dtype=torch.float32).view(-1, 1)

    mean = X.mean(dim=0)
    std = X.std(dim=0)
    std = torch.where(std < 1e-8, torch.ones_like(std), std)
    X = (X - mean) / std

    n = len(X)
    indices = torch.randperm(n)
    n_val = max(1, int(n * val_ratio))
    val_idx = indices[:n_val]
    train_idx = indices[n_val:]

    train_ds = TensorDataset(X[train_idx], y[train_idx])
    val_ds = TensorDataset(X[val_idx], y[val_idx])

    return train_ds, val_ds, (mean, std)


# ---------------------------------------------------------------------------
# Training
# ---------------------------------------------------------------------------

def train_epoch(model: nn.Module, loader: DataLoader, optimizer: optim.Optimizer,
                criterion: nn.Module) -> float:
    model.train()
    total_loss = 0.0
    for batch_x, batch_y in loader:
        optimizer.zero_grad()
        pred = model(batch_x)
        loss = criterion(pred, batch_y)
        loss.backward()
        optimizer.step()
        total_loss += loss.item() * batch_x.size(0)
    return total_loss / max(1, len(loader.dataset))


@torch.no_grad()
def evaluate(model: nn.Module, loader: DataLoader, criterion: nn.Module,
             denorm_target_std: float = 1.0) -> dict:
    model.eval()
    total_loss = 0.0
    all_abs_errors = []
    for batch_x, batch_y in loader:
        pred = model(batch_x)
        loss = criterion(pred, batch_y)
        total_loss += loss.item() * batch_x.size(0)
        abs_err = (pred - batch_y).abs()
        all_abs_errors.extend(abs_err.squeeze().tolist())
    n = max(1, len(loader.dataset))
    return {
        "loss": total_loss / n,
        "mae_norm": sum(all_abs_errors) / len(all_abs_errors),
    }


def train(args: argparse.Namespace):
    train_ds, val_ds, (feat_mean, feat_std) = load_csv(
        args.data, val_ratio=args.val_ratio
    )
    print(f"train samples: {len(train_ds)}  val samples: {len(val_ds)}", file=sys.stderr)

    train_loader = DataLoader(train_ds, batch_size=args.batch_size, shuffle=True)
    val_loader = DataLoader(val_ds, batch_size=args.batch_size, shuffle=False)

    model = FusionMLP(input_dim=9, hidden1=args.hidden1, hidden2=args.hidden2)
    criterion = nn.SmoothL1Loss(beta=args.huber_beta)  # Huber loss
    optimizer = optim.Adam(model.parameters(), lr=args.lr,
                           weight_decay=args.weight_decay)
    scheduler = optim.lr_scheduler.ReduceLROnPlateau(
        optimizer, mode="min", factor=0.5, patience=args.scheduler_patience
    )

    best_val_loss = float("inf")
    best_state = None
    patience_counter = 0

    for epoch in range(1, args.epochs + 1):
        train_loss = train_epoch(model, train_loader, optimizer, criterion)
        val_metrics = evaluate(model, val_loader, criterion)
        scheduler.step(val_metrics["loss"])

        if epoch <= 5 or epoch % 10 == 0 or epoch == args.epochs:
            print(f"epoch {epoch:4d}  train_loss={train_loss:.6f}  "
                  f"val_loss={val_metrics['loss']:.6f}  "
                  f"val_mae_norm={val_metrics['mae_norm']:.4f}", file=sys.stderr)

        if val_metrics["loss"] < best_val_loss:
            best_val_loss = val_metrics["loss"]
            best_state = {k: v.clone() for k, v in model.state_dict().items()}
            patience_counter = 0
        else:
            patience_counter += 1
            if patience_counter >= args.early_stop:
                print(f"early stop at epoch {epoch}", file=sys.stderr)
                break

    if best_state is not None:
        model.load_state_dict(best_state)

    return model, feat_mean, feat_std, best_val_loss


# ---------------------------------------------------------------------------
# Export C++ header with hardcoded weights
# ---------------------------------------------------------------------------

def export_cpp_header(model: FusionMLP, feat_mean: torch.Tensor, feat_std: torch.Tensor,
                      out_path: str) -> None:
    """Export model weights as a C++ header for zero-dependency inference."""
    layers: List[Tuple[torch.Tensor, torch.Tensor]] = []
    params_dict = dict(model.named_parameters())
    for name, param in model.named_parameters():
        if name.endswith(".weight"):
            bias_name = name.replace(".weight", ".bias")
            bias = params_dict[bias_name].detach()
            layers.append((param.detach(), bias))

    lines = []
    lines.append("// Auto-generated by train_fusion_mlp.py — do not edit")
    lines.append("#pragma once")
    lines.append("")
    lines.append("#include <array>")
    lines.append("#include <cmath>")
    lines.append("")
    lines.append("namespace rk3588::modules {")
    lines.append("")
    lines.append("// MLP: 9 → 32 → 16 → 1, HuberLoss-trained distance predictor")
    lines.append("struct FusionMLPWeights {")
    lines.append(f"    static constexpr int kInputDim = {model.input_dim};")
    lines.append(f"    static constexpr int kHidden1 = {layers[0][0].size(0)};")
    lines.append(f"    static constexpr int kHidden2 = {layers[1][0].size(0)};")
    lines.append(f"    static constexpr int kOutputDim = 1;")
    lines.append("")

    # feature normalization constants
    lines.append("    // feature mean (9,)")
    lines.append("    static constexpr float kFeatMean[9] = {")
    vals = ", ".join(f"{v:.8f}f" for v in feat_mean.tolist())
    lines.append(f"        {vals}")
    lines.append("    };")
    lines.append("")
    lines.append("    // feature std (9,)")
    lines.append("    static constexpr float kFeatStd[9] = {")
    vals = ", ".join(f"{v:.8f}f" for v in feat_std.tolist())
    lines.append(f"        {vals}")
    lines.append("    };")
    lines.append("")

    layer_names = ["fc1", "fc2", "fc3"]
    for layer_idx, (weight, bias) in enumerate(layers):
        name = layer_names[layer_idx]
        out_dim, in_dim = weight.shape

        # weights: row-major C array
        lines.append(f"    // {name}: weight [{out_dim}x{in_dim}]")
        lines.append(f"    static constexpr float k{name.capitalize()}Weight"
                     f"[{out_dim}][{in_dim}] = {{")
        for row in range(out_dim):
            row_vals = ", ".join(f"{weight[row, col]:.8f}f" for col in range(in_dim))
            lines.append(f"        {{{row_vals}}},")
        lines.append("    };")
        lines.append("")

        # bias
        lines.append(f"    // {name}: bias [{out_dim}]")
        lines.append(f"    static constexpr float k{name.capitalize()}Bias[{out_dim}] = {{")
        vals = ", ".join(f"{bias[i]:.8f}f" for i in range(out_dim))
        lines.append(f"        {vals}")
        lines.append("    };")
        lines.append("")

    lines.append("};")
    lines.append("")

    # inference function
    lines.append("inline float mlpFusionPredict(const float features[9]) {")
    lines.append("    // normalize")
    lines.append("    float x[9];")
    lines.append("    for (int i = 0; i < 9; ++i) {")
    lines.append("        x[i] = (features[i] - FusionMLPWeights::kFeatMean[i]) / FusionMLPWeights::kFeatStd[i];")
    lines.append("    }")
    lines.append("")
    lines.append("    // fc1: 9 → 32 + ReLU")
    lines.append("    float h1[32];")
    lines.append("    for (int i = 0; i < 32; ++i) {")
    lines.append("        float sum = FusionMLPWeights::kFc1Bias[i];")
    lines.append("        for (int j = 0; j < 9; ++j) {")
    lines.append("            sum += FusionMLPWeights::kFc1Weight[i][j] * x[j];")
    lines.append("        }")
    lines.append("        h1[i] = sum > 0.0f ? sum : 0.0f;  // ReLU")
    lines.append("    }")
    lines.append("")
    lines.append("    // fc2: 32 → 16 + ReLU")
    lines.append("    float h2[16];")
    lines.append("    for (int i = 0; i < 16; ++i) {")
    lines.append("        float sum = FusionMLPWeights::kFc2Bias[i];")
    lines.append("        for (int j = 0; j < 32; ++j) {")
    lines.append("            sum += FusionMLPWeights::kFc2Weight[i][j] * h1[j];")
    lines.append("        }")
    lines.append("        h2[i] = sum > 0.0f ? sum : 0.0f;  // ReLU")
    lines.append("    }")
    lines.append("")
    lines.append("    // fc3: 16 → 1 (no activation)")
    lines.append("    float sum = FusionMLPWeights::kFc3Bias[0];")
    lines.append("    for (int j = 0; j < 16; ++j) {")
    lines.append("        sum += FusionMLPWeights::kFc3Weight[0][j] * h2[j];")
    lines.append("    }")
    lines.append("    return sum;")
    lines.append("}")
    lines.append("")
    lines.append("}  // namespace rk3588::modules")
    lines.append("")

    with open(out_path, "w", encoding="utf-8") as f:
        f.write("\n".join(lines))
    print(f"exported C++ header to {out_path}", file=sys.stderr)


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Train lightweight MLP fusion head for distance correction"
    )
    parser.add_argument("--data", required=True,
                        help="CSV training data from extract_fusion_training_data.py")
    parser.add_argument("--out-dir", default="/tmp/fusion_mlp_model",
                        help="Output directory for model and C++ header")
    parser.add_argument("--epochs", type=int, default=200,
                        help="Max training epochs")
    parser.add_argument("--batch-size", type=int, default=64,
                        help="Batch size")
    parser.add_argument("--lr", type=float, default=0.001,
                        help="Learning rate")
    parser.add_argument("--weight-decay", type=float, default=1e-5,
                        help="L2 regularization")
    parser.add_argument("--huber-beta", type=float, default=0.5,
                        help="Huber loss beta (transition from L2 to L1)")
    parser.add_argument("--val-ratio", type=float, default=0.15,
                        help="Validation split ratio")
    parser.add_argument("--hidden1", type=int, default=32,
                        help="Hidden layer 1 size")
    parser.add_argument("--hidden2", type=int, default=16,
                        help="Hidden layer 2 size")
    parser.add_argument("--early-stop", type=int, default=40,
                        help="Early stopping patience (epochs)")
    parser.add_argument("--scheduler-patience", type=int, default=15,
                        help="LR scheduler patience")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if not os.path.exists(args.data):
        print(f"error: data file not found: {args.data}", file=sys.stderr)
        return 2

    os.makedirs(args.out_dir, exist_ok=True)

    print(f"Training FusionMLP on {args.data}", file=sys.stderr)
    model, feat_mean, feat_std, best_loss = train(args)

    # save PyTorch checkpoint
    ckpt_path = os.path.join(args.out_dir, "fusion_mlp.pt")
    torch.save({
        "model_state_dict": model.state_dict(),
        "feat_mean": feat_mean,
        "feat_std": feat_std,
        "input_dim": model.input_dim,
        "hidden1": args.hidden1,
        "hidden2": args.hidden2,
    }, ckpt_path)
    print(f"saved checkpoint to {ckpt_path}", file=sys.stderr)

    # export C++ header
    hpp_path = os.path.join(args.out_dir, "learned_fusion_corrector_weights.hpp")
    export_cpp_header(model, feat_mean, feat_std, hpp_path)

    # final benchmark on a few samples
    print("\n--- sample predictions ---")
    model.eval()
    data_rows = []
    with open(args.data, "r") as f:
        reader = csv.DictReader(f)
        for row in reader:
            data_rows.append(row)
    if data_rows:
        for _ in range(min(8, len(data_rows))):
            row = random.choice(data_rows)
            feats = torch.tensor([[float(row[n]) for n in FEATURE_NAMES]], dtype=torch.float32)
            feats_norm = (feats - feat_mean) / feat_std
            target = float(row["target_distance_m"])
            with torch.no_grad():
                pred = model(feats_norm).item()
            print(f"  {row['class_name']:12s}  "
                  f"pred={pred:.3f}m  target={target:.3f}m  "
                  f"err={abs(pred-target):.3f}m  "
                  f"raw={float(row['raw_distance_m']):.3f}m")

    return 0


if __name__ == "__main__":
    sys.exit(main())
