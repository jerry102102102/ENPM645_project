"""Cross-episode neighborhood diagnostics, not training or a Bayes-error estimate."""

import argparse
import json
from pathlib import Path

import numpy as np

from phase1b.expert.data import load_arrays, validate_sidecar
from phase1b.interfaces import pose

PROFILES = ("ft_object", "ft_object_joints", "ft_object_joints_tactile")
OBJECT_FIELDS = (
    "object_position",
    "object_orientation",
    "object_linear_velocity",
    "object_angular_velocity",
)


def windows(
    episodes,
    *,
    profile,
    history_length,
    stride=1,
    ft_source="contact_reconstructed",
    max_windows=500,
):
    """episodes: (group_id, observations, candidate_trace); only causal public keys."""
    if profile not in PROFILES or ft_source not in (
        "contact_reconstructed",
        "wrist_transmitted",
    ):
        raise ValueError("explicit observation profile and F/T source required")
    if (
        type(history_length) is not int
        or history_length <= 0
        or type(stride) is not int
        or stride <= 0
    ):
        raise ValueError("positive history length and stride required")
    prefix = "wrist_ft_" if ft_source == "contact_reconstructed" else "wrist_sensor_"
    keys = [prefix + "left", prefix + "right", *OBJECT_FIELDS]
    if profile != "ft_object":
        keys += ["arm_joint_position", "arm_joint_velocity"]
    if profile == "ft_object_joints_tactile":
        keys += ["tactile_left", "tactile_right"]
    X = []
    Y = []
    groups = []
    times = []
    for group, obs, labels in episodes:
        if not isinstance(group, str) or not group:
            raise ValueError("episode/group ID required")
        t = np.asarray(obs["timestamp"])
        if (
            t.ndim != 1
            or len(t) == 0
            or not np.isfinite(t).all()
            or not np.allclose(np.diff(t), 0.01, atol=1e-12, rtol=0)
        ):
            raise ValueError("consecutive 100 Hz timestamps required")
        np.testing.assert_array_equal(t, labels["timestamp"])
        if np.asarray(labels["candidate_valid"]).dtype != bool or np.asarray(
            labels["candidate_valid"]
        ).shape != (len(t),):
            raise ValueError("aligned bool candidate mask required")
        if np.asarray(labels["candidate_assist_torque_ref"]).shape != (len(t),):
            raise ValueError("aligned candidate torque required")
        parts = []
        for key in keys:
            v = np.array(obs[key], dtype=float, copy=True)
            expected = {
                prefix + "left": (6,),
                prefix + "right": (6,),
                "object_position": (3,),
                "object_orientation": (4,),
                "object_linear_velocity": (3,),
                "object_angular_velocity": (3,),
                "arm_joint_position": (2, 7),
                "arm_joint_velocity": (2, 7),
                "tactile_left": (2, 3, 4, 4),
                "tactile_right": (2, 3, 4, 4),
            }[key]
            if v.shape != (len(t), *expected) or not np.isfinite(v).all():
                raise ValueError("bad public feature " + key)
            if key == "object_orientation":
                for i in range(len(t)):
                    pose(np.r_[np.zeros(3), v[i]])
                    if v[i, 0] < 0:
                        v[i] *= -1  # same rotation, quaternion double cover
            parts.append(v.reshape(len(t), -1))
        features = np.concatenate(parts, axis=1)
        for end in range(history_length - 1, len(t), stride):
            if not labels["candidate_valid"][end]:
                continue
            label = float(labels["candidate_assist_torque_ref"][end])
            if not np.isfinite(label):
                raise ValueError("nonfinite candidate label")
            X.append(features[end - history_length + 1 : end + 1].reshape(-1))
            Y.append(label)
            groups.append(group)
            times.append(t[end])
            if len(X) > max_windows:
                raise ValueError(
                    "pilot window limit exceeded; increase stride or explicitly change max_windows"
                )
    if not X:
        raise ValueError("no complete valid windows")
    return dict(
        features=np.array(X),
        labels=np.array(Y),
        groups=np.array(groups),
        timestamps=np.array(times),
        profile=profile,
        ft_source=ft_source,
        history_length=history_length,
        feature_keys=keys,
    )


def assess(reference, query, *, radius, torque_tolerance_nm, min_neighbors=3):
    for key in ("profile", "ft_source", "history_length", "feature_keys"):
        if reference[key] != query[key]:
            raise ValueError("reference/query feature contract mismatch")
    if (
        not np.isfinite(radius)
        or radius <= 0
        or not np.isfinite(torque_tolerance_nm)
        or torque_tolerance_nm <= 0
        or type(min_neighbors) is not int
        or min_neighbors < 2
    ):
        raise ValueError(
            "positive radius/tolerance and at least two neighbors required"
        )
    for dataset in (reference, query):
        features = np.asarray(dataset["features"])
        if (
            features.ndim != 2
            or not all(features.shape)
            or not np.isfinite(features).all()
        ):
            raise ValueError("nonempty finite feature matrix required")
        for key in ("labels", "groups", "timestamps"):
            if np.asarray(dataset[key]).shape != (len(features),):
                raise ValueError("window count mismatch: " + key)
        if (
            not np.isfinite(dataset["labels"]).all()
            or not np.isfinite(dataset["timestamps"]).all()
        ):
            raise ValueError("nonfinite label or timestamp")
    ref, queries = reference["features"], query["features"]
    if ref.ndim != 2 or queries.ndim != 2 or ref.shape[1] != queries.shape[1]:
        raise ValueError("window shape mismatch")
    scale = np.std(
        ref, axis=0
    )  # Fit only the declared reference split; never query/test data.
    scale[scale < 1e-12] = (
        1.0  # Constant features use one native unit; report this convention.
    )
    rows = []
    # ponytail: exact O(N_ref*N_query*D), bounded by windows(); index neighbors only beyond pilot size.
    for x, y, group, t in zip(
        queries, query["labels"], query["groups"], query["timestamps"]
    ):
        distance = np.sqrt(np.mean(((ref - x) / scale) ** 2, axis=1))
        mask = (distance <= radius) & (reference["groups"] != group)
        local = reference["labels"][mask]
        row = dict(
            timestamp=float(t),
            episode_id=str(group),
            neighbors=int(mask.sum()),
            matched=len(local) >= min_neighbors,
        )
        if row["matched"]:
            row.update(
                local_label_std_nm=float(np.std(local)),
                label_range_nm=float(np.ptp(local)),
                squared_error_to_neighbor_mean=float((y - local.mean()) ** 2),
                inconsistent=bool(np.ptp(np.r_[local, y]) > torque_tolerance_nm),
                sign_conflict=bool(
                    abs(y) > torque_tolerance_nm
                    and np.any((local * y < 0) & (np.abs(local) > torque_tolerance_nm))
                ),
            )
        rows.append(row)
    matched = [r for r in rows if r["matched"]]
    return dict(
        status="diagnostic_only" if matched else "insufficient_matched_evidence",
        profile=query["profile"],
        ft_source=query["ft_source"],
        history_length=query["history_length"],
        radius_standardized_rms=radius,
        torque_tolerance_nm=torque_tolerance_nm,
        min_neighbors=min_neighbors,
        reference_windows=len(ref),
        query_windows=len(queries),
        matched_windows=len(matched),
        coverage=len(matched) / len(rows),
        mean_local_label_std_nm=float(
            np.mean([r["local_label_std_nm"] for r in matched])
        )
        if matched
        else None,
        inconsistency_fraction=float(np.mean([r["inconsistent"] for r in matched]))
        if matched
        else None,
        sign_conflict_fraction=float(np.mean([r["sign_conflict"] for r in matched]))
        if matched
        else None,
        scaling="reference-split std only; constant dimensions use one native unit",
        interpretation="Finite-neighborhood variation proxy, not conditional variance, irreducible error, or proof of learnability; candidate labels only.",
        windows=rows,
    )


def from_sidecars(paths, expert_id, **kwargs):
    if expert_id not in ("E0", "E1", "E2"):
        raise ValueError("choose one expert; never pool E1/E2 targets")
    episodes = []
    for path in paths:
        path = Path(path)
        validate_sidecar(path)
        meta = json.loads((path / "metadata.json").read_text())
        observation_paths = [
            p for p in meta["source_files"] if Path(p).name == "observations.npz"
        ]
        if len(observation_paths) != 1:
            raise ValueError("one source observation stream required")
        episodes.append(
            (
                meta["source_files"][
                    observation_paths[0]
                ],  # identical source cannot become another split by renaming it
                load_arrays(observation_paths[0]),
                load_arrays(path / "expert_internal" / f"{expert_id}.npz"),
            )
        )
    return windows(episodes, **kwargs)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--reference", type=Path, nargs="+", required=True)
    parser.add_argument("--query", type=Path, nargs="+", required=True)
    parser.add_argument("--expert", choices=["E0", "E1", "E2"], required=True)
    parser.add_argument("--profile", choices=PROFILES, required=True)
    parser.add_argument(
        "--ft-source",
        choices=["contact_reconstructed", "wrist_transmitted"],
        default="contact_reconstructed",
    )
    parser.add_argument("--history", type=int, required=True)
    parser.add_argument("--stride", type=int, default=1)
    parser.add_argument("--radius", type=float, required=True)
    parser.add_argument("--torque-tolerance-nm", type=float, required=True)
    parser.add_argument("--min-neighbors", type=int, default=3)
    parser.add_argument("--output", type=Path, required=True)
    a = parser.parse_args()
    if a.output.exists():
        raise FileExistsError(a.output)
    opts = dict(
        profile=a.profile,
        history_length=a.history,
        stride=a.stride,
        ft_source=a.ft_source,
    )
    report = assess(
        from_sidecars(a.reference, a.expert, **opts),
        from_sidecars(a.query, a.expert, **opts),
        radius=a.radius,
        torque_tolerance_nm=a.torque_tolerance_nm,
        min_neighbors=a.min_neighbors,
    )
    report["expert_id"] = a.expert
    a.output.parent.mkdir(parents=True, exist_ok=True)
    a.output.write_text(json.dumps(report, indent=2, allow_nan=False) + "\n")
    print(report["status"])


if __name__ == "__main__":
    main()
