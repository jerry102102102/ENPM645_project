"""One command: collect, validate, replay, regress, plot, report; never overwrite results."""

import argparse
import io
import json
import subprocess
import sys
import unittest
from pathlib import Path

import matplotlib
import numpy as np

matplotlib.use("Agg")
import matplotlib.pyplot as plt

from phase1a.data.episode_io import DatasetReader
from phase1a.scripts.validate_dataset import regression, validate_smoke
from phase1a.simulation.scenario_loader import (
    DEPENDENCIES,
    REPO,
    config_hash,
    digest,
    load_scenario,
)


def figures(root):
    dest = root / "figures"
    dest.mkdir()
    r = DatasetReader(root / "smoke/reference_test")
    o = r.sections["observations"]
    c = r.sections["controller"]
    t = o["timestamp"]
    fig, ax = plt.subplots(2, 1, figsize=(9, 6), sharex=True)
    ax[0].step(t, c["desired_assist_torque_ref"], where="post")
    ax[0].set_ylabel("Desired extra torque (Nm)")
    ax[0].set_title("Scripted input accepted; support_only does not execute it")
    ax[1].plot(
        r.sections["next_observations"]["timestamp"],
        c["realized_object_contact_wrench"][:, 4],
        color="tab:orange",
    )
    ax[1].set_ylabel("Total contact torque Y (Nm)")
    ax[1].set_xlabel("Episode time (s)")
    ax[1].set_title(
        "Measured total contact torque: not isolated assistance, not a tracking metric"
    )
    for a in ax:
        a.grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(dest / "reference_and_total_contact.png", dpi=160)
    plt.close(fig)
    fig, ax = plt.subplots(figsize=(9, 4))
    k = 100
    ax.scatter(
        [t[k]], [2], marker="s", label="o_t + reference decision (same simulation time)"
    )
    ax.scatter(
        c["controller_call_timestamps"][k],
        np.ones(5),
        marker=">",
        label="Actual command application",
    )
    ax.scatter(
        c["realized_substep_timestamps"][k],
        np.zeros(5),
        marker="o",
        label="Post-step measurement",
    )
    ax.axvline(
        r.sections["next_observations"]["timestamp"][k],
        color="gray",
        linestyle=":",
        label="o_next / next decision boundary",
    )
    ax.set(
        yticks=[0, 1, 2],
        yticklabels=["Measure", "Apply", "Decide"],
        xlabel="Episode time (s)",
        title="One transition contains five 2 ms physics / controller steps",
        ylim=(-0.5, 3.7),
    )
    ax.legend(loc="upper left", fontsize=8)
    ax.grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(dest / "transition_timing.png", dpi=160)
    plt.close(fig)
    fig, axes = plt.subplots(3, 2, figsize=(10, 8), sharex=True)
    for row, name in enumerate("ABC"):
        data = DatasetReader(root / "smoke" / name).sections["observations"]
        t = data["timestamp"]
        for side in ("left", "right"):
            axes[row, 0].plot(
                t, np.linalg.norm(data["wrist_ft_" + side][:, :3], axis=1), label=side
            )
            axes[row, 1].plot(
                t,
                np.linalg.norm(data["tactile_" + side].reshape(len(t), -1), axis=1),
                label=side,
            )
        axes[row, 0].set_ylabel(name + " force norm (N)")
        axes[row, 1].set_ylabel(name + " tactile norm (N)")
        for a in axes[row]:
            a.grid(alpha=0.3)
    axes[0, 0].set_title("Independent local wrist F/T")
    axes[0, 1].set_title("Existing angular tactile sensor bins")
    axes[0, 0].legend()
    axes[-1, 0].set_xlabel("Episode time (s)")
    axes[-1, 1].set_xlabel("Episode time (s)")
    fig.tight_layout()
    fig.savefig(dest / "sensor_smoke.png", dpi=160)
    plt.close(fig)


def report(root, result):
    cfg = load_scenario()
    d = result["datasets"]
    replay = sum(
        v["exact_arrays_including_nan_masks"]
        for v in result["regression"]["deterministic_replay"].values()
    )
    maxerr = max(
        max(v["max_absolute_errors"].values())
        for v in result["regression"]["original_v2_comparison"].values()
    )
    text = """# Phase 1A — Frozen Scenario and Data Pipeline

**Data Pipeline GO · Controller Integration Ready.** Desired Torque Tracking、Expert Label 與 Offline Learning 尚未通過；本階段不要求這三項。

本報告由 `python -m phase1a.scripts.run_all` 依實際驗證產生。這是 CPU MuJoCo 模擬結果，沒有新增力分配器或 expert。

## 執行結果

| Episode | Transitions | Physics steps | Expert valid | Assist executed |
|---|---:|---:|---:|---:|
"""
    for name, v in d.items():
        text += f"| {name} | {v['transitions']} | {v['physics_steps']} | {v['expert_valid_count']} | {v['assist_executed_count']} |\n"
    text += f"""
- 10 項 contract / causal isolation / corruption rejection tests 通過（詳見 `unit-tests.txt`）。
- 四個 episodes 重跑，共 **{replay} 個 NPZ arrays 完全一致**，包含 NaN missing-value masks；只承諾本次 runtime 的確定性。
- 5 秒正負 reference 與零 reference 的 sensor trajectories、實際命令及接觸 wrench 完全一致，證實 support-only 沒有執行新指令。
- 原 v2 A/B/C 重新執行：位置、姿態、速度、root commands、命令施加後接觸 wrench 的最大絕對差 **{maxerr:.3g}**（各欄位單位詳見 validation.json；閾值 1e-9）。
- 原 v2 接觸移除、關閉 drives、等合力及 timestep-halving controls 再次通過。
- 最大線性力平衡殘差 **{max(v["max_balance_residual_N"] for v in d.values()):.3g} N**；最大接觸 penetration **{max(v["max_penetration_m"] for v in d.values()):.6f} m**；無 MuJoCo warnings。

## Q1–Q9

**Q1 場景凍結：已實作、已驗證。** 從已驗證 v2 block 0 的實際 compiled model 擷取，固定 geometry、接觸參數、初始姿態、human 和 disturbance。registry 同時驗證 config、MJCF、依賴原始碼及來源 metadata；變更需新版本。

**Q2 三個握持點：已驗證。** Panel-local left = [-0.2, 0, 0] m；right = [0.2, 0, 0] m；human = [0.27, -0.08, 0] m。左右相對 nominal quaternion 都是 [1, 0, 0, 0]。這些是名義幾何；gripper roots 仍自由運動，contact points 由物理解算。

**Q3 力矩語意：已實作、已驗證。** 額外 assistance，world +Y、瞬時 panel COM、Nm、右手定則。既有 `rotate_y` 與此一致：+X 側在正 pitch 時朝 -Z 移動；+X 點向 -Z 施力產生正 Y torque。測試幅度 ±0.02 Nm 僅驗證資料傳遞，未證明可追蹤。

**Q4 分離：已實作、已驗證。** observations / next_observations / supervision / controller / privileged 分檔。policy builder 只允許十項 sensor features，拒絕未來時間與額外欄位；毒化未來、控制與 privileged arrays 不影響當下 history。Object pose 暫用 MuJoCo state，屬明確允許的輸入；scenario/human goal 不允許。

**Q5 時序與讀寫：已實作、已驗證。** 100 Hz reference/logging、500 Hz controller/physics；每筆 transition 保存五個命令、physics ticks 與前後 timestamps，驗證 observation ≤ decision < next observation，並驗證相鄰 transition 連續。o_t 是舊命令持續作用時的量測；新命令施加後的同時間 force response 僅存 controller diagnostics。

**Q6 Reference 接入：已實作、已驗證。** 0、正、0、負、0 的測試訊號經 typed interface 傳入 adapter。source=scripted_test；assist_reference_executed=false；tracking error=NaN 且 mask=false；expert label=NaN 且 invalid。

**Q7 Baseline：已驗證。** 重用原 v2 MJCF、human/disturbance、support_commands 與 tactile。回歸量化結果見上方及 validation.json；未修改舊版原始碼、報告與資料。

**Q8 組員接入：介面已實作、接入機制已驗證。** 見 repository 的 `docs/CONTROL_HANDOFF.md`。使用 `--controller module:Class` 替換；測試已用獨立 delegating class 完成短 episode、儲存和驗證，無須修改核心 pipeline。此測試只證明接入，並未驗證另一個力分配控制器。

**Q9 正式 demonstrations 還缺什麼：尚未實作。** Track B 需完成額外 torque + baseline 的合成、左右接觸力分配、內力/飽和管理及 requested→allocated→executed→realized 的追蹤驗證；需定義 baseline 與 assistance 的可驗證分離方式。之後 Track A 才建立 model-based expert、label validity gates、示範品質篩選與專用 collection mode。現有 `LearnedForcePolicy` 僅是介面，尚無模型、訓練或有效 teacher。

## Evidence and provenance

- Scenario: `{cfg["scenario_id"]}@{cfg["scenario_version"]}`
- model_hash: `{cfg["model_hash"]}`
- config_hash: `{config_hash(cfg)}`
- 完整驗證：`validation.json`；各 episode：`smoke/{{A,B,C,reference_test}}/`；欄位字典：`smoke/schema.json`。
- 圖表：`figures/reference_and_total_contact.png`、`figures/transition_timing.png`、`figures/sensor_smoke.png`。
- 重現：在 repository root 執行 `uv run --locked python -m phase1a.scripts.run_all --output results/phase1a/reproduction`，選未存在的輸出位置。

這是 UMI-style 的 observation/action/transition 分離設計，並非 UMI 原始格式。現有 root-drive commands 是 MuJoCo-specific actuator adapter；policy features / desired object torque 不依賴特定機械臂關節配置。
"""
    (root / "report.md").write_text(text)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--output", type=Path, default=REPO / "results/phase1a")
    a = ap.parse_args()
    root = a.output.resolve()
    for name in ("smoke", "validation.json", "unit-tests.txt", "figures", "report.md"):
        if (root / name).exists():
            ap.error("refusing to overwrite " + str(root / name))
    root.mkdir(parents=True, exist_ok=True)
    frozen = {p: digest(REPO / p) for p in DEPENDENCIES}
    subprocess.run(
        [
            sys.executable,
            "-m",
            "phase1a.scripts.run_pipeline_smoke",
            "--output",
            str(root / "smoke"),
        ],
        check=True,
        cwd=REPO,
    )
    stream = io.StringIO()
    suite = unittest.defaultTestLoader.loadTestsFromName("phase1a.tests.test_contracts")
    tests = unittest.TextTestRunner(stream=stream, verbosity=2).run(suite)
    (root / "unit-tests.txt").write_text(stream.getvalue())
    if not tests.wasSuccessful():
        raise RuntimeError(stream.getvalue())
    print(
        "Contract and causal isolation tests passed; replay/regression running.",
        flush=True,
    )
    result = {
        "datasets": validate_smoke(root / "smoke"),
        "unit_tests": {"passed": True, "count": tests.testsRun},
        "regression": regression(root / "smoke"),
    }
    assert frozen == {p: digest(REPO / p) for p in DEPENDENCIES}
    result.update(
        data_pipeline_go=True,
        controller_integration_ready=True,
        desired_torque_tracking="unavailable",
        expert_labels="invalid",
        frozen_dependencies_unchanged=True,
    )
    (root / "validation.json").write_text(json.dumps(result, indent=2) + "\n")
    figures(root)
    report(root, result)
    print(
        "Data Pipeline GO; Controller Integration Ready. Report: "
        + str(root / "report.md"),
        flush=True,
    )


if __name__ == "__main__":
    main()
