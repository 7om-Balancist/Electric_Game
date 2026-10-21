import importlib.util
import json
from pathlib import Path


ROOT = Path(r"D:\E_Game_2026\Vision_Code")
PROJECTS = {
    "Q4": ROOT / "ec_vision_h_visual_state8_uart2_v6.12.34_q4_imu_curve_soft",
    "Q5": ROOT / "ec_vision_h_visual_state8_uart2_v6.12.37_q5_startup_triggered",
}

BALL_KEYS = (
    "roi_enable", "roi_x", "roi_y", "roi_w", "roi_h",
    "r_min", "r_nom", "r_max", "acquire_q_min", "track_q_min",
    "confirm_frames", "coast_frames", "lost_frames", "conf_min",
    "cv_track_local", "cv_track_half_w", "cv_track_half_max_w",
    "cv_track_local_miss", "cv_recover_local_miss", "gate_px",
    "cv_y_hard_gate_px", "cv_blue_enable", "cv_blue_acquire_enable",
    "cv_blue_track_enable", "cv_blue_recover_enable", "cv_blue_cc_enable",
    "cv_pipe_veto_enable", "cv_edge_zone_px", "cv_edge_min_visible",
    "cv_motion_enable", "cv_rail_auto", "cv_fast_track_enable",
)

MOTOR_KEYS = (
    "uart_baud", "address", "pulses_per_rev", "hard_limit_deg",
    "rpm", "acceleration", "command_period_ms", "query_period_ms",
    "ack_required", "ack_timeout_ms", "max_timeouts",
    "position_arm_timeout_ms", "position_stale_ms", "position_lost_fault_ms",
)

COMMON_CONTROL_KEYS = (
    "default_mode", "min_confidence", "vision_loss_zero_ms",
    "vision_loss_disable_ms", "arm_vision_ready_ms", "go_guard_ms",
    "start_zero_band_px", "start_zero_speed_px_s", "start_zero_hold_ms",
    "direction_sign", "auto_direction", "finish_hold_ms",
)


def load_defaults(path: Path, module_name: str):
    spec = importlib.util.spec_from_file_location(module_name, path / "params_h.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.DEFAULTS


def subset(mapping, keys):
    return {key: mapping.get(key) for key in keys if key in mapping}


for label, project in PROJECTS.items():
    defaults = load_defaults(project, f"params_{label.lower()}")
    control = defaults["control"]
    selected = {
        "global": defaults["global"],
        "q3": defaults["q3"],
        "ball": subset(defaults["ball"], BALL_KEYS),
        "motor": subset(defaults["motor"], MOTOR_KEYS),
        "car": defaults.get("car", {}),
        "control_common": subset(control, COMMON_CONTROL_KEYS),
        "control_q3": {key: value for key, value in control.items()
                       if key.startswith("q3_pd_") or key in (
                           "q3_low_angle_enable", "q3_direct_motor_enable")},
        "control_q4": {key: value for key, value in control.items()
                       if key.startswith("q4_")},
        "control_q5": {key: value for key, value in control.items()
                       if key.startswith("q5_")},
        "log": defaults.get("log", {}),
    }
    print(f"===== {label} =====")
    print(json.dumps(selected, ensure_ascii=False, indent=2, sort_keys=True))
