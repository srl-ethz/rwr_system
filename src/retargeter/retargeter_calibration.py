#!/usr/bin/env python3

"""
Offline calibration script for MANO adjustments using a collected dataset.

Inputs:
    - One or more JSON files produced by retargeter_collect_calibration_data.py.
      Each JSON contains samples with:
          { "mano_keypoints": [[x,y,z], ...], "joint_angles": [...] }
    - A hand name (e.g. "orcahand_v1b").
      URDF, scheme, mano_adjustment, and retargeter_cfg paths are resolved like
      in the teleop / record_demonstration launch files.

Outputs:
    - Updated mano_adjustment YAML where the global "all" translation/rotation/scale
      have been optimized to minimize retargeting error.

The script uses Bayesian optimization with a Gaussian process (scikit-learn) and
Expected Improvement to tune 9 parameters:
    all.translation: tx, ty, tz
    all.rotation:    rx, ry, rz
    all.scale:       sx, sy, sz
"""

import argparse
import json
import os
from pathlib import Path
from tqdm import tqdm

import numpy as np
import torch
import yaml
from scipy.stats import norm
from sklearn.gaussian_process import GaussianProcessRegressor
from sklearn.gaussian_process.kernels import ConstantKernel as C, Matern, WhiteKernel
import matplotlib.pyplot as plt

from faive_system.src.retargeter.retargeter import Retargeter


def load_dataset(json_paths):
    """Load samples from one or more JSON files."""
    mano_list = []
    joints_list = []
    for p in json_paths:
        with open(p, "r") as f:
            data = json.load(f)
        for s in data.get("samples", []):
            mano = np.asarray(s.get("mano_keypoints", []), dtype=np.float32)
            joints = np.asarray(s.get("joint_angles", []), dtype=np.float32)
            if mano.size == 0 or joints.size == 0:
                continue
            mano_list.append(mano)
            joints_list.append(joints)
    if not mano_list:
        raise ValueError("No valid samples found in provided JSON paths.")
    return mano_list, joints_list


def build_retargeter(urdf_path, scheme_path, mano_adjustments, retargeter_cfg):
    """Construct a Retargeter with given config."""
    device = "cuda" if torch.cuda.is_available() else "cpu"
    return Retargeter(
        device="cuda",
        mjcf_filepath=None,
        urdf_filepath=str(urdf_path),
        hand_scheme=str(scheme_path),
        mano_adjustments=mano_adjustments,
        retargeter_cfg=retargeter_cfg,
        optimizer="RMSprop",
    )


def theta_to_mano_adjustments(base_adjustments, theta):
    """
    Map theta to the mano_adjustments dict.

    Layout:
        For each optimized block (e.g. 'all', 'thumb', 'index', ...), we use
        9 parameters:
            [tx, ty, tz, rx, ry, rz, sx, sy, sz]
    """
    adj = dict(base_adjustments)  # shallow copy of top-level
    return adj  # placeholder, real mapping added in make_objective


def build_layout(optimize_fingers):
    """
    Build a layout describing how theta slices map to adjustment blocks.

    Always includes 'all', then any fingers from optimize_fingers
    (subset of ['thumb', 'index', 'middle', 'ring', 'pinky']).
    """
    blocks = ["all"] + list(optimize_fingers)
    layout = []
    for idx, name in enumerate(blocks):
        start = idx * 9
        end = start + 9
        layout.append((name, start, end))
    return layout


def apply_theta_to_adjustments(base_adjustments, theta, layout):
    """
    Apply theta to the given base_adjustments according to layout.
    """
    adj = dict(base_adjustments)
    for block_name, i0, i1 in layout:
        tx, ty, tz, rx, ry, rz, sx, sy, sz = theta[i0:i1]
        block = dict(adj.get(block_name, {}))
        block["translation"] = [float(tx), float(ty), float(tz)]
        block["rotation"] = [float(rx), float(ry), float(rz)]
        block["scale"] = [float(sx), float(sy), float(sz)]
        adj[block_name] = block
    return adj


def make_objective(retargeter, mano_list, joints_list, urdf_path, scheme_path, base_adjustments, retargeter_cfg, layout):
    """
    Return an objective function f(theta) -> scalar MSE over the dataset.
    """

    def objective(theta):
        mano_adj = apply_theta_to_adjustments(base_adjustments, theta, layout)
        retargeter.mano_adjustments = mano_adj
        retargeter.clean()
        sq_errors = []
        for mano_pts, q_gt in zip(mano_list, joints_list):
            debug_dict = {}
            try:
                q_pred, debug_dict = retargeter.retarget(mano_pts, debug_dict)
            except Exception as e:
                # If something blows up for this theta, penalize heavily.
                print(f"Error in objective: {e}")
                return 1e6
            q_pred = np.asarray(q_pred, dtype=np.float32)
            if q_pred.shape != q_gt.shape:
                print(f"Error in objective: q_pred.shape != q_gt.shape")
                return 1e6
            diff = q_pred - q_gt
            sq_errors.append(np.mean(diff ** 2))
        return float(np.mean(sq_errors))

    return objective


def evaluate_errors(
    mano_list,
    joints_list,
    urdf_path: Path,
    scheme_path: Path,
    mano_adjustments: dict,
    retargeter_cfg: dict,
) -> np.ndarray:
    """
    Evaluate per-sample MSE errors for a given mano_adjustments dict.
    Returns an array of shape (n_samples,).
    """
    retargeter = build_retargeter(urdf_path, scheme_path, mano_adjustments, retargeter_cfg)
    errs = []
    for mano_pts, q_gt in zip(mano_list, joints_list):
        debug_dict = {}
        try:
            q_pred, debug_dict = retargeter.retarget(mano_pts, debug_dict)
        except Exception:
            errs.append(1e6)
            continue
        q_pred = np.asarray(q_pred, dtype=np.float32)
        if q_pred.shape != q_gt.shape:
            errs.append(1e6)
            continue
        diff = q_pred - q_gt
        errs.append(float(np.mean(diff ** 2)))
    return np.asarray(errs, dtype=np.float64)


def suggest_next_point(gp, X, y, bounds, n_candidates=4096, random_state=None):
    """Suggest next theta by maximizing Expected Improvement."""
    rng = np.random.default_rng(random_state)
    dim = bounds.shape[0]
    lows = bounds[:, 0]
    highs = bounds[:, 1]

    X_cand = rng.uniform(lows, highs, size=(n_candidates, dim))
    mu, sigma = gp.predict(X_cand, return_std=True)
    sigma = sigma.reshape(-1)
    f_best = np.min(y)
    improvement = f_best - mu
    with np.errstate(divide="warn"):
        Z = improvement / sigma
        ei = improvement * norm.cdf(Z) + sigma * norm.pdf(Z)
        ei[sigma == 0.0] = 0.0
    idx = int(np.argmax(ei))
    return X_cand[idx]


def bayesian_optimisation(
    objective,
    bounds,
    n_init=8,
    n_iter=32,
    random_state=None,
):
    """Simple BO loop using sklearn GP + EI, with normalized GP input space."""
    rng = np.random.default_rng(random_state)
    dim = bounds.shape[0]
    lows = bounds[:, 0]
    highs = bounds[:, 1]

    def normalize(x):
        """Normalize x in original bounds to [0, 1]^d."""
        return (x - lows) / (highs - lows)

    def denormalize(xn):
        """Map from [0, 1]^d back to original space."""
        return xn * (highs - lows) + lows

    # Initial random design (in normalized space)
    X_orig = rng.uniform(lows, highs, size=(n_init, dim))
    X = normalize(X_orig)
    y = np.array([objective(x_orig) for x_orig in X_orig], dtype=np.float64)

    # Debug / diagnostics: track EI at each BO step and best-so-far objective.
    ei_hist = []
    best_so_far_hist = [float(np.min(y))]

    for _ in tqdm(range(n_iter)):
        kernel = (
            C(1.0, (1e-3, 1e3))
            * Matern(
                length_scale=np.ones(dim),
                length_scale_bounds=(1e-3, 1e3),
                nu=2.5
            )
            + WhiteKernel(noise_level=1e-2, noise_level_bounds=(1e-8, 1e1))
        )

        gp = GaussianProcessRegressor(
            kernel=kernel,
            alpha=1e-8,
            normalize_y=True,
            n_restarts_optimizer=5,
            random_state=0,
        )        
        gp.fit(X, y)

        # Suggest a candidate in normalized space
        def suggest_next_point_normalized(gp, X, y, random_state=None, n_candidates=4096, bounds=None):
            # bounds should be None: we sample from [0,1]^dim
            rng_ = np.random.default_rng(random_state)
            X_cand = rng_.uniform(0.0, 1.0, size=(n_candidates, dim))
            mu, sigma = gp.predict(X_cand, return_std=True)
            sigma = sigma.reshape(-1)
            f_best = np.min(y)
            improvement = f_best - mu 
            with np.errstate(divide="warn"):
                Z = improvement / sigma
                ei = improvement * norm.cdf(Z) + sigma * norm.pdf(Z)
                ei[sigma == 0.0] = 0.0
            idx = int(np.argmax(ei))
            return X_cand[idx]

        x_next_normalized = suggest_next_point_normalized(gp, X, y, random_state=rng)
        x_next = denormalize(x_next_normalized)

        # Compute Expected Improvement at the chosen next normalized point for logging.
        mu_next, sigma_next = gp.predict(x_next_normalized[None, :], return_std=True)
        mu_next = float(mu_next[0])
        sigma_next = float(sigma_next[0])
        f_best = float(np.min(y))
        if sigma_next > 0.0:
            Z = (f_best - mu_next) / sigma_next
            ei_next = (f_best - mu_next) * norm.cdf(Z) + sigma_next * norm.pdf(Z)
        else:
            ei_next = 0.0
        ei_hist.append(float(ei_next))

        y_next = objective(x_next)

        X = np.vstack([X, x_next_normalized])
        y = np.concatenate([y, [y_next]])
        best_so_far_hist.append(float(min(best_so_far_hist[-1], y_next)))

    # Return original space values for result and history (not normalized)
    best_idx = int(np.argmin(y))
    # X is normalized space, to return in original space
    X_orig_all = denormalize(X)
    return (
        X_orig_all[best_idx],  # best x in original space
        y[best_idx],           # best y
        X_orig_all,            # all X samples in original space
        y,
        np.asarray(ei_hist, dtype=np.float64),
        np.asarray(best_so_far_hist, dtype=np.float64)
    )


def parse_args():
    parser = argparse.ArgumentParser(
        description="Calibrate MANO adjustments using BO on a collected dataset."
    )
    parser.add_argument(
        "--dataset",
        "-d",
        nargs="+",
        required=True,
        help="Path(s) to JSON dataset file(s) from retargeter_collect_calibration_data.py.",
    )
    parser.add_argument(
        "--hand-name",
        default="orcahand_v1b",
        help=(
            "Hand name, e.g. 'orcahand_v1b', 'orca_v1', or 'mimic'. "
            "URDF, scheme, and cfg paths are resolved like in the launch files."
        ),
    )
    parser.add_argument(
        "--optimize-fingers",
        nargs="*",
        default=[],
        choices=["thumb", "index", "middle", "ring", "pinky"],
        help=(
            "Additional fingers (besides the global 'all' block) whose "
            "translation/rotation/scale will also be optimized."
        ),
    )
    parser.add_argument(
        "--output",
        "-o",
        default=None,
        help="Output mano_adjustment YAML path. Defaults to base file with '_calibrated' suffix.",
    )
    parser.add_argument(
        "--init-samples",
        type=int,
        default=30,
        help="Number of random initial samples for BO.",
    )
    parser.add_argument(
        "--iterations",
        type=int,
        default=100,
        help="Number of BO iterations after initialization.",
    )
    parser.add_argument(
        "--seed",
        type=int,
        default=0,
        help="Random seed for reproducibility.",
    )
    return parser.parse_args()


def main():
    args = parse_args()

    json_paths = [Path(p) for p in args.dataset]
    mano_list, joints_list = load_dataset(json_paths)

    hand_name = args.hand_name

    # Resolve base directories relative to this file so paths match the launch files.
    retargeter_dir = Path(__file__).resolve().parent  # .../src/retargeter
    project_root = retargeter_dir.parent.parent  # faive_system/ (faive_system package root)
    viz_dir = project_root / "src" / "viz"
    experiments_dir = project_root / "experiments"

    if "mimic" in hand_name:
        urdf_path = (
            viz_dir / "models" / "mimic" / "converted.urdf"
        )
        scheme_path = (
            viz_dir / "models" / hand_name / "scheme.yaml"
        )
    elif "orca" in hand_name:
        urdf_path = (
            viz_dir / "models" / hand_name / "urdf" / f"{hand_name}.urdf"
        )
        scheme_path = (
            viz_dir / "models" / hand_name / f"scheme_{hand_name}.yaml"
        )
    else:
        raise NotImplementedError(f"hand_name '{hand_name}' not supported")

    mano_adjustment_path = (
        experiments_dir / "cfgs" / "retargeter" / f"mano_adjustment_{hand_name}.yaml"
    )
    retargeter_cfg_path = (
        experiments_dir / "cfgs" / "retargeter" / f"retargeter_cfgs_{hand_name}.yaml"
    )

    with mano_adjustment_path.open("r") as f:
        base_adjustments = yaml.safe_load(f)
    with retargeter_cfg_path.open("r") as f:
        retargeter_cfg = yaml.safe_load(f)

    # Build parameter layout: always 'all', plus any selected fingers.
    layout = build_layout(args.optimize_fingers)

    retargeter = build_retargeter(urdf_path, scheme_path, base_adjustments, retargeter_cfg)
    objective = make_objective(
        retargeter,
        mano_list,
        joints_list,
        urdf_path,
        scheme_path,
        base_adjustments,
        retargeter_cfg,
        layout,
    )

    # Bounds for [tx, ty, tz, rx, ry, rz, sx, sy, sz] per optimized block.
    single_bounds = np.array(
        [
            [-0.02, 0.02],  # tx
            [-0.02, 0.02],  # ty
            [-0.02, 0.02],  # tz
            [-0.2, 0.2],    # rx
            [-0.2, 0.2],    # ry
            [-0.2, 0.2],    # rz
            [0.8, 1.2],     # sx
            [0.8, 1.2],     # sy
            [0.8, 1.2],     # sz
        ],
        dtype=np.float64,
    )
    bounds = np.tile(single_bounds, (len(layout), 1))

    (
        best_theta,
        best_val,
        X_hist,
        y_hist,
        ei_hist,
        best_so_far_hist,
    ) = bayesian_optimisation(
        objective,
        bounds,
        n_init=args.init_samples,
        n_iter=args.iterations,
        random_state=args.seed,
    )

    print(f"Best objective value: {best_val:.6f}")
    print(f"Best theta: {best_theta}")

    best_adjustments = apply_theta_to_adjustments(base_adjustments, best_theta, layout)

    # Visualize fit before/after optimization (per-sample MSE histogram).
    base_errs = evaluate_errors(
        mano_list,
        joints_list,
        urdf_path,
        scheme_path,
        base_adjustments,
        retargeter_cfg,
    )
    best_errs = evaluate_errors(
        mano_list,
        joints_list,
        urdf_path,
        scheme_path,
        best_adjustments,
        retargeter_cfg,
    )

    print(
        f"Baseline MSE: mean={base_errs.mean():.6f}, "
        f"median={np.median(base_errs):.6f}"
    )
    print(
        f"Optimized MSE: mean={best_errs.mean():.6f}, "
        f"median={np.median(best_errs):.6f}"
    )

    if args.output is not None:
        out_path = Path(args.output)
    else:
        out_path = mano_adjustment_path.with_name(
            mano_adjustment_path.stem + "_calibrated.yaml"
        )

    with out_path.open("w") as f:
        yaml.safe_dump(best_adjustments, f)

    print(f"Saved calibrated MANO adjustments to {out_path}")

    # Save a simple histogram plot comparing per-sample errors before/after,
    # plus BO diagnostics (objective trace, EI trace, and sampling locations).
    try:
        # 1) Histogram of per-sample errors before/after calibration.
        fig_hist, ax_hist = plt.subplots(figsize=(6, 4))
        ax_hist.hist(
            base_errs,
            bins=30,
            alpha=0.5,
            label="before",
        )
        ax_hist.hist(
            best_errs,
            bins=30,
            alpha=0.5,
            label="after",
        )
        ax_hist.set_xlabel("Per-sample MSE")
        ax_hist.set_ylabel("Count")
        ax_hist.set_title(f"Retargeter calibration fit ({hand_name})")
        ax_hist.legend()
        fig_hist.tight_layout()

        hist_path = out_path.with_suffix(".png")
        fig_hist.savefig(hist_path)
        plt.close(fig_hist)
        print(f"Saved fit visualization to {hist_path}")

        # 2) Objective (MSE) trace over BO iterations.
        fig_obj, ax_obj = plt.subplots(figsize=(6, 4))
        iters = np.arange(len(y_hist))
        ax_obj.plot(iters, y_hist, marker="o", linestyle="-", label="observed objective")
        ax_obj.plot(
            np.arange(len(best_so_far_hist)),
            best_so_far_hist,
            marker=".",
            linestyle="--",
            label="best-so-far objective",
        )
        ax_obj.set_xlabel("BO evaluation index")
        ax_obj.set_ylabel("MSE objective")
        ax_obj.set_title("BO objective during calibration")
        ax_obj.legend()
        fig_obj.tight_layout()

        obj_path = out_path.with_name(out_path.stem + "_bo_objective.png")
        fig_obj.savefig(obj_path)
        plt.close(fig_obj)
        print(f"Saved BO objective trace to {obj_path}")

        # 3) Expected Improvement trace for the chosen next points.
        fig_ei, ax_ei = plt.subplots(figsize=(6, 4))
        ei_iters = np.arange(len(ei_hist))
        ax_ei.plot(ei_iters, ei_hist, marker="o", linestyle="-")
        ax_ei.set_xlabel("BO iteration index")
        ax_ei.set_ylabel("Expected Improvement at x_next")
        ax_ei.set_title("Expected Improvement during calibration")
        fig_ei.tight_layout()

        ei_path = out_path.with_name(out_path.stem + "_bo_ei.png")
        fig_ei.savefig(ei_path)
        plt.close(fig_ei)
        print(f"Saved BO EI trace to {ei_path}")

        # 4) Where BO is sampling in parameter space.
        dim = X_hist.shape[1]
        n_params_to_plot = min(dim, 6)
        n_cols = 3
        n_rows = int(np.ceil(n_params_to_plot / n_cols))
        fig_samp, axes = plt.subplots(
            n_rows, n_cols, figsize=(4 * n_cols, 3 * n_rows), sharex=True
        )
        axes = np.atleast_1d(axes).reshape(-1)
        for j in range(n_params_to_plot):
            ax = axes[j]
            ax.plot(
                iters,
                X_hist[:, j],
                marker="o",
                linestyle="-",
                markersize=2,
            )
            ax.set_ylabel(f"theta[{j}]")
        for k in range(n_params_to_plot, len(axes)):
            axes[k].axis("off")
        axes[0].set_title("Sampling locations per parameter")
        axes[-1].set_xlabel("BO evaluation index")
        fig_samp.tight_layout()

        samp_path = out_path.with_name(out_path.stem + "_bo_samples.png")
        fig_samp.savefig(samp_path)
        plt.close(fig_samp)
        print(f"Saved BO sampling diagnostics to {samp_path}")
    except Exception as e:
        print(f"Failed to save fit visualization: {e}")


if __name__ == "__main__":
    main()
