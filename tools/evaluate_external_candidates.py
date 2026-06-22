from __future__ import annotations

import argparse
import csv
import os
import re
import shutil
import subprocess
import sys
import time
from dataclasses import dataclass
from pathlib import Path


PYTHON_EXE = Path(r"C:\Users\myway\.cache\codex-runtimes\codex-primary-runtime\dependencies\python\python.exe")
RUNTIME_DIR = Path(r"C:\SSAFY_RACE\MyCar_Python\Bot_Python")
SIMULATOR_EXE = Path(r"C:\SSAFY_RACE\Simulator\Algo.exe")
SETTINGS_FILE = Path("settings/settings-speed.json")
AIRSIM_SETTINGS = Path.home() / "Documents" / "AirSim" / "settings.json"


@dataclass(frozen=True)
class Candidate:
    name: str
    source: Path


CANDIDATES = [
    Candidate("local_v15_recovery_tuned", Path("python/my_car.py")),
    Candidate("oogg7754_game_ssafy_race", Path("external_sources/oogg7754_Game/싸피레이스.py")),
    Candidate("windy825_speed", Path("external_sources/windy825_Airsim/speed.py")),
    Candidate("windy825_ssafy", Path("external_sources/windy825_Airsim/ssafy.py")),
    Candidate("windy825_ssafy2", Path("external_sources/windy825_Airsim/ssafy2.py")),
    Candidate("windy825_my_car", Path("external_sources/windy825_Airsim/my_car.py")),
    Candidate("windy825_basic", Path("external_sources/windy825_Airsim/basic.py")),
    Candidate("windy825_germany", Path("external_sources/windy825_Airsim/germany.py")),
    Candidate("windy825_lim", Path("external_sources/windy825_Airsim/lim.py")),
    Candidate("windy825_ob_middle", Path("external_sources/windy825_Airsim/ob_middle.py")),
    Candidate("windy825_seong", Path("external_sources/windy825_Airsim/seong.py")),
    Candidate("yuj1818_til_4th", Path("external_sources/yuj1818_TIL/codingtest/싸피레이스_4등.py")),
    Candidate("leesh125_ssafy_race", Path("external_sources/leesh125_SSAFY_8th_Class/Seasonal_Semester/SSAFY_race/my_car.py")),
    Candidate("ehddn5252_template_bot", Path("external_sources/ehddn5252_ssafy_race/Template_Python/Bot_Python/my_car.py")),
    Candidate("caerinso_my_car1", Path("external_sources/caerinso_SSAFY_RACE/my_car1.py")),
    Candidate("noranbanana_root_my_car", Path("external_sources/noranbanana_SSAFY_RACE/my_car.py")),
    Candidate("noranbanana_hojong_my_car", Path("external_sources/noranbanana_SSAFY_RACE/hojong/Bot_Python/my_car.py")),
    Candidate("noranbanana_kim_my_car", Path("external_sources/noranbanana_SSAFY_RACE/김현명/my_car.py")),
    Candidate("no_but_gunha_my_car", Path("external_sources/No-but-why-not-this_SSAFY_RACE/MyCar_20251112/Template_Python/gunha_Python/my_car.py")),
    Candidate("no_but_gunha_runcode_speed", Path("external_sources/No-but-why-not-this_SSAFY_RACE/MyCar_20251112/Template_Python/gunha_Python/runcode_speed.py")),
    Candidate("no_but_yusin_my_car", Path("external_sources/No-but-why-not-this_SSAFY_RACE/MyCar_20251112/Template_Python/yusin_Python/my_car.py")),
    Candidate("no_but_yusin_71_ssafy_final", Path("external_sources/No-but-why-not-this_SSAFY_RACE/MyCar_20251112/Template_Python/yusin_Python/71/ssafy_final.py")),
    Candidate("no_but_yusin_71_v17_scoped", Path("external_sources/No-but-why-not-this_SSAFY_RACE/MyCar_20251112/Template_Python/yusin_Python/71/zzz_gpt_dev_v17_v9base_post20_scoped.py")),
    Candidate("no_but_yusin_71_v16_cornerfix", Path("external_sources/No-but-why-not-this_SSAFY_RACE/MyCar_20251112/Template_Python/yusin_Python/71/zzz_gpt_dev_v16_from_v9_post20_cornerfix.py")),
    Candidate("no_but_yusin_71_v15_minitune", Path("external_sources/No-but-why-not-this_SSAFY_RACE/MyCar_20251112/Template_Python/yusin_Python/71/zzz_gpt_dev_v15_from_v9_post20_minitune.py")),
    Candidate("no_but_yusin_71_rightangle_v3", Path("external_sources/No-but-why-not-this_SSAFY_RACE/MyCar_20251112/Template_Python/yusin_Python/71/zzz_gpt_dev_v9_max170_rightangle_v3.py")),
    Candidate("no_but_yusin_71_v12", Path("external_sources/No-but-why-not-this_SSAFY_RACE/MyCar_20251112/Template_Python/yusin_Python/71/runcode_ssafy_v12.py")),
    Candidate("no_but_yusin_71_v10_6_map_3_2", Path("external_sources/No-but-why-not-this_SSAFY_RACE/MyCar_20251112/Template_Python/yusin_Python/71/runcode_ssafy_v10_6_map_3_2.py")),
    Candidate("no_but_yusin_71_v9", Path("external_sources/No-but-why-not-this_SSAFY_RACE/MyCar_20251112/Template_Python/yusin_Python/71/runcode_ssafy_v9.py")),
    Candidate("no_but_yusin_71_safe_tune_v9_fix1", Path("external_sources/No-but-why-not-this_SSAFY_RACE/MyCar_20251112/Template_Python/yusin_Python/71/runcode_ssafy_v2_safe_tune_v9_passlock_fix1.py")),
    Candidate("no_but_yusin_71_basic_final61_v4", Path("external_sources/No-but-why-not-this_SSAFY_RACE/MyCar_20251112/Template_Python/yusin_Python/71/basic_final61_v4.py")),
    Candidate("no_but_yusin_161_v14_3_fix", Path("external_sources/No-but-why-not-this_SSAFY_RACE/MyCar_20251112/Template_Python/yusin_Python/161map/basic_gpt_first015_v14_3_FIX.py")),
    Candidate("no_but_yusin_161_basic_final_v4", Path("external_sources/No-but-why-not-this_SSAFY_RACE/MyCar_20251112/Template_Python/yusin_Python/161map/basic_final_v4.py")),
    Candidate("no_but_yusin_31_stable4", Path("external_sources/No-but-why-not-this_SSAFY_RACE/MyCar_20251112/Template_Python/yusin_Python/31map/basic_gpt_ver015_v2_stable_4.py")),
    Candidate("no_but_yusin_10_basic_ver019", Path("external_sources/No-but-why-not-this_SSAFY_RACE/MyCar_20251112/Template_Python/yusin_Python/10map/basic_ver019.py")),
    Candidate("nonulti_template_bot", Path("external_sources/NonUlti_Algorithm/SSAFY2학기/사피레이스/Template_Python/Bot_Python/my_car.py")),
]


RUNNER_SHIM = r'''

# --- Codex external-candidate runner shim ---
def _codex_get_player_name(self, _json_data):
    self.player_name = "Car1"
    print("[DrivingController] Player name : {}".format(self.player_name))


def _codex_set_player_name(self):
    return "Car1"


try:
    DrivingClient.getPlayerName = _codex_get_player_name
    DrivingClient.set_player_name = _codex_set_player_name
except NameError:
    pass


try:
    _codex_original_control_driving = DrivingClient.control_driving

    def _codex_control_driving(self, car_controls, sensing_info):
        result = _codex_original_control_driving(self, car_controls, sensing_info)
        telemetry_file = _codex_os.environ.get("SSAFY_TELEMETRY_FILE", "")
        if telemetry_file:
            tick = getattr(self, "_codex_tick", 0) + 1
            self._codex_tick = tick
            obstacle_dist = ""
            obstacle_middle = ""
            if getattr(sensing_info, "track_forward_obstacles", None):
                obstacle = sensing_info.track_forward_obstacles[0]
                if isinstance(obstacle, dict):
                    obstacle_dist = obstacle.get("dist", "")
                    obstacle_middle = obstacle.get("to_middle", "")
                else:
                    obstacle_dist = getattr(obstacle, "dist", "")
                    obstacle_middle = getattr(obstacle, "to_middle", "")

            command = result if result is not None else car_controls
            header_needed = not _codex_os.path.exists(telemetry_file)
            with open(telemetry_file, "a", encoding="utf-8") as log_file:
                if header_needed:
                    log_file.write(
                        "tick,progress,speed,to_middle,moving_angle,collided,obstacle_dist,"
                        "obstacle_to_middle,steering,throttle,brake\n"
                    )
                log_file.write(
                    "{},{:.4f},{:.4f},{:.4f},{:.4f},{},{},{},{:.4f},{:.4f},{:.4f}\n".format(
                        tick,
                        float(getattr(sensing_info, "lap_progress", 0.0)),
                        float(getattr(sensing_info, "speed", 0.0)),
                        float(getattr(sensing_info, "to_middle", 0.0)),
                        float(getattr(sensing_info, "moving_angle", 0.0)),
                        int(bool(getattr(sensing_info, "collided", False))),
                        obstacle_dist,
                        obstacle_middle,
                        float(getattr(command, "steering", 0.0)),
                        float(getattr(command, "throttle", 0.0)),
                        float(getattr(command, "brake", 0.0)),
                    )
                )
        return result

    DrivingClient.control_driving = _codex_control_driving
except NameError:
    pass
# --- End Codex runner shim ---
'''


def read_text(path: Path) -> str:
    for encoding in ("utf-8-sig", "utf-8", "cp949"):
        try:
            return path.read_text(encoding=encoding)
        except UnicodeDecodeError:
            continue
    return path.read_text(errors="replace")


def normalize_source(text: str) -> str:
    text = text.replace("from Interface.drive_controller import DrivingController", "from DrivingInterface.drive_controller import DrivingController")
    text = text.replace("from drive_controller import DrivingController", "from DrivingInterface.drive_controller import DrivingController")
    text = text.replace("from bcrypt import kdf\n", "")
    text = text.replace("from sklearn.cluster import k_means\n", "")
    if "import os as _codex_os" not in text:
        text = "import os as _codex_os\n" + text
    return text


def inject_shim(text: str) -> str:
    if "Codex external-candidate runner shim" in text:
        return text
    match = re.search(r"(?m)^if\s+__name__\s*==\s*['\"]__main__['\"]\s*:", text)
    if match:
        return text[: match.start()] + RUNNER_SHIM + "\n" + text[match.start() :]
    return text + RUNNER_SHIM + "\n\nif __name__ == \"__main__\":\n    client = DrivingClient()\n    raise SystemExit(client.run())\n"


def prepare_candidates(names: set[str] | None) -> list[Path]:
    prepared_root = Path("external_candidates")
    prepared_root.mkdir(parents=True, exist_ok=True)
    prepared_files: list[Path] = []
    for candidate in CANDIDATES:
        if names and candidate.name not in names:
            continue
        if not candidate.source.exists():
            continue
        target_dir = prepared_root / candidate.name
        target_dir.mkdir(parents=True, exist_ok=True)
        source_text = normalize_source(read_text(candidate.source))
        target = target_dir / "my_car.py"
        target.write_text(inject_shim(source_text), encoding="utf-8")
        prepared_files.append(target)
    return prepared_files


def candidate_name(path: Path) -> str:
    return path.parent.name


def compile_candidate(path: Path) -> tuple[bool, str]:
    completed = subprocess.run(
        [str(PYTHON_EXE), "-m", "py_compile", str(path)],
        text=True,
        capture_output=True,
        check=False,
    )
    output = (completed.stdout + completed.stderr).strip()
    return completed.returncode == 0, output


def kill_simulator() -> None:
    for image in ("Algo-Win64-Shipping.exe", "Algo.exe"):
        subprocess.run(["taskkill", "/F", "/IM", image, "/T"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, check=False)


def summarize_telemetry(path: Path) -> dict[str, object]:
    if not path.exists():
        return {"telemetry": False}

    with path.open(newline="", encoding="utf-8") as stream:
        rows = list(csv.DictReader(stream))
    if not rows:
        return {"telemetry": True, "rows": 0}

    def as_float(row: dict[str, str], key: str) -> float:
        value = row.get(key, "")
        return float(value) if value != "" else 0.0

    stable_rows = rows[5:] if len(rows) > 5 else rows
    progress = [as_float(row, "progress") for row in rows]
    speed = [as_float(row, "speed") for row in rows]
    middle = [as_float(row, "to_middle") for row in stable_rows]
    collided = [int(row.get("collided", "0") or 0) for row in stable_rows]
    brake = [as_float(row, "brake") for row in rows]
    throttle = [as_float(row, "throttle") for row in rows]
    return {
        "telemetry": True,
        "rows": len(rows),
        "max_progress": round(max(progress), 2),
        "last_progress": round(progress[-1], 2),
        "max_speed": round(max(speed), 2),
        "max_abs_middle": round(max(abs(value) for value in middle), 2) if middle else 0.0,
        "penalty_rows": sum(abs(value) > 10.0 for value in middle),
        "collision_rows": sum(collided),
        "brake_rows": sum(value > 0.01 for value in brake),
        "throttle_cut_rows": sum(value < 0.5 for value in throttle),
    }


def run_candidate(path: Path, seconds: float, startup_seconds: float) -> dict[str, object]:
    logs_root = Path("race_logs") / "external_candidates" / candidate_name(path)
    logs_root.mkdir(parents=True, exist_ok=True)
    telemetry = logs_root / "telemetry.csv"
    stdout_file = logs_root / "stdout.txt"
    stderr_file = logs_root / "stderr.txt"
    for artifact in (telemetry, stdout_file, stderr_file):
        if artifact.exists():
            artifact.unlink()

    shutil.copy2(path, RUNTIME_DIR / "my_car.py")
    AIRSIM_SETTINGS.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(SETTINGS_FILE, AIRSIM_SETTINGS)

    kill_simulator()
    simulator = subprocess.Popen(
        [str(SIMULATOR_EXE), "-ResX=640", "-ResY=480", "-windowed"],
        cwd=SIMULATOR_EXE.parent,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    )
    time.sleep(startup_seconds)

    env = os.environ.copy()
    env["SSAFY_TELEMETRY_FILE"] = str(telemetry.resolve())
    started = time.perf_counter()
    timed_out = False
    return_code: int | None = None
    stdout = ""
    stderr = ""
    try:
        completed = subprocess.run(
            [str(PYTHON_EXE), "my_car.py"],
            cwd=RUNTIME_DIR,
            env=env,
            text=True,
            capture_output=True,
            timeout=seconds,
            check=False,
        )
        return_code = completed.returncode
        stdout = completed.stdout
        stderr = completed.stderr
    except subprocess.TimeoutExpired as error:
        timed_out = True
        stdout = error.stdout or ""
        stderr = error.stderr or ""
    finally:
        elapsed = time.perf_counter() - started
        if simulator.poll() is None:
            simulator.terminate()
        kill_simulator()

    stdout_file.write_text(stdout, encoding="utf-8", errors="replace")
    stderr_file.write_text(stderr, encoding="utf-8", errors="replace")
    summary = summarize_telemetry(telemetry)
    summary.update(
        {
            "name": candidate_name(path),
            "return_code": return_code,
            "timed_out": timed_out,
            "elapsed": round(elapsed, 2),
            "stdout_tail": " | ".join(stdout.splitlines()[-3:]),
            "stderr_tail": " | ".join(stderr.splitlines()[-3:]),
        }
    )
    return summary


def print_compile_results(prepared: list[Path]) -> list[Path]:
    runnable: list[Path] = []
    print("compile results")
    print(f"{'candidate':34} {'ok':>3} error")
    print("-" * 90)
    for path in prepared:
        ok, output = compile_candidate(path)
        print(f"{candidate_name(path):34} {'yes' if ok else 'no':>3} {output.splitlines()[-1] if output else ''}")
        if ok:
            runnable.append(path)
    return runnable


def print_run_results(results: list[dict[str, object]]) -> None:
    print("\nrun results")
    print(
        f"{'candidate':34} {'rows':>5} {'prog':>6} {'spd':>6} {'mid':>5} "
        f"{'pen':>4} {'col':>4} {'brk':>4} {'tout':>5} {'rc':>4}"
    )
    print("-" * 90)
    for result in results:
        print(
            f"{str(result.get('name')):34} "
            f"{str(result.get('rows', '-')):>5} "
            f"{str(result.get('max_progress', '-')):>6} "
            f"{str(result.get('max_speed', '-')):>6} "
            f"{str(result.get('max_abs_middle', '-')):>5} "
            f"{str(result.get('penalty_rows', '-')):>4} "
            f"{str(result.get('collision_rows', '-')):>4} "
            f"{str(result.get('brake_rows', '-')):>4} "
            f"{'yes' if result.get('timed_out') else 'no':>5} "
            f"{str(result.get('return_code')):>4}"
        )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--names", nargs="*", help="Candidate names to prepare/run. Default: all known candidates.")
    parser.add_argument("--prepare-only", action="store_true")
    parser.add_argument("--compile-only", action="store_true")
    parser.add_argument("--seconds", type=float, default=25.0)
    parser.add_argument("--startup-seconds", type=float, default=9.0)
    parser.add_argument("--max-candidates", type=int, default=0)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    names = set(args.names) if args.names else None
    prepared = prepare_candidates(names)
    if not prepared:
        print("No candidates prepared.", file=sys.stderr)
        return 2

    runnable = print_compile_results(prepared)
    if args.prepare_only or args.compile_only:
        return 0 if len(runnable) == len(prepared) else 1

    if args.max_candidates > 0:
        runnable = runnable[: args.max_candidates]

    results = []
    for path in runnable:
        print(f"\n>>> running {candidate_name(path)} for {args.seconds:.0f}s")
        results.append(run_candidate(path, args.seconds, args.startup_seconds))
        print_run_results(results[-1:])

    print_run_results(results)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
