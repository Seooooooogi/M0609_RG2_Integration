#!/usr/bin/env python3
"""
inspect_episode.py — Inspect a LeRobot episode parquet file.

Usage:
  python3 tools/inspect_episode.py <parquet_path> [--plot]

Examples:
  python3 tools/inspect_episode.py ~/lerobot_datasets/hammer/data/chunk-000/episode_000000.parquet
  python3 tools/inspect_episode.py ~/lerobot_datasets/hammer/data/chunk-000/episode_000000.parquet --plot
"""
import argparse
import numpy as np
import pandas as pd
from pathlib import Path


# RG2 kinematic constants (from OnRobotRGControllerServer.py)
_RG2_L1     = 0.108505
_RG2_L3     = 0.055
_RG2_THETA1 = 1.41371
_RG2_THETA3 = 0.76794
_RG2_DY     = -0.0144

JOINT_NAMES = ['j1', 'j2', 'j3', 'j4', 'j5', 'j6']


def gripper_angle_to_width_mm(angle: float) -> float:
    """finger_joint angle (rad) → gripper opening width (mm)."""
    return (np.cos(angle + _RG2_THETA3) * _RG2_L3
            + _RG2_DY + _RG2_L1 * np.cos(_RG2_THETA1)) * 2 * 1000


def inspect(parquet_path: Path, plot: bool = False):
    df = pd.read_parquet(parquet_path)

    ts = df['timestamp'].values
    dt = np.diff(ts)
    fps_avg = 1.0 / dt.mean() if dt.mean() > 0 else 0.0

    print(f"File    : {parquet_path}")
    print(f"Frames  : {len(df)}")
    print(f"Duration: {ts[-1] - ts[0]:.3f}s  ({fps_avg:.1f} fps avg)")
    print(f"Columns : {df.columns.tolist()}")
    print()

    states  = np.stack(df['observation.state'].values)  # (N, D)
    actions = np.stack(df['action'].values)              # (N, D)
    n_dims  = states.shape[1]
    has_gripper = n_dims >= 7

    # ── Arm joints ──────────────────────────────────────────────────────────
    print("=== observation.state — arm joints (degrees) ===")
    header = f"  {'joint':<6}  {'min':>8}  {'max':>8}  {'range':>8}  {'mean':>8}"
    print(header)
    for i in range(min(6, n_dims)):
        deg = np.degrees(states[:, i])
        print(f"  {JOINT_NAMES[i]:<6}  {deg.min():>7.1f}°  {deg.max():>7.1f}°"
              f"  {deg.ptp():>7.1f}°  {deg.mean():>7.1f}°")

    # ── Gripper width (7th dim) ──────────────────────────────────────────────
    if has_gripper:
        widths_mm = np.array([gripper_angle_to_width_mm(a) for a in states[:, 6]])
        print(f"  {'grip':<6}  {widths_mm.min():>7.1f}mm {widths_mm.max():>7.1f}mm"
              f"  {widths_mm.ptp():>7.1f}mm {widths_mm.mean():>7.1f}mm")
    else:
        print("  (gripper width not recorded — observation.state has 6 dims)")

    # ── Action delta ─────────────────────────────────────────────────────────
    diff = np.abs(states - actions)
    print()
    print("=== action - state mean abs diff ===")
    for i in range(min(6, n_dims)):
        print(f"  {JOINT_NAMES[i]}: {np.degrees(diff[:, i]).mean():.4f}°")
    if has_gripper:
        grip_diff_mm = np.array([gripper_angle_to_width_mm(a) for a in diff[:, 6]])
        print(f"  grip: {grip_diff_mm.mean():.4f} mm")

    # ── Plot ─────────────────────────────────────────────────────────────────
    if plot:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt

        n_rows, n_cols = (2, 4) if has_gripper else (2, 3)
        fig, axes = plt.subplots(n_rows, n_cols, figsize=(4 * n_cols, 5), sharex=True)
        fig.suptitle(parquet_path.name, fontsize=10)
        axes_flat = axes.flat

        for i in range(min(6, n_dims)):
            ax = next(axes_flat)
            ax.plot(ts, np.degrees(states[:, i]),  label='state', lw=1)
            ax.plot(ts, np.degrees(actions[:, i]), '--', alpha=0.6, label='action', lw=1)
            ax.set_title(JOINT_NAMES[i], fontsize=9)
            ax.set_ylabel('deg', fontsize=8)
            ax.legend(fontsize=7)

        if has_gripper:
            ax = next(axes_flat)
            state_w  = [gripper_angle_to_width_mm(a) for a in states[:, 6]]
            action_w = [gripper_angle_to_width_mm(a) for a in actions[:, 6]]
            ax.plot(ts, state_w,  label='state', lw=1)
            ax.plot(ts, action_w, '--', alpha=0.6, label='action', lw=1)
            ax.set_title('gripper width', fontsize=9)
            ax.set_ylabel('mm', fontsize=8)
            ax.legend(fontsize=7)

        # Hide unused axes
        for ax in axes_flat:
            ax.set_visible(False)

        plt.tight_layout()
        out = parquet_path.with_suffix('.png')
        plt.savefig(out, dpi=100)
        print(f"\nPlot saved: {out}")


def main():
    parser = argparse.ArgumentParser(description="Inspect a LeRobot episode parquet file")
    parser.add_argument('parquet', type=Path, help='Path to episode_XXXXXX.parquet')
    parser.add_argument('--plot', action='store_true', help='Save joint trajectory PNG next to parquet')
    args = parser.parse_args()
    inspect(args.parquet.expanduser(), plot=args.plot)


if __name__ == '__main__':
    main()
