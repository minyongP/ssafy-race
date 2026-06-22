from __future__ import annotations

import argparse
import shutil
import subprocess
import sys
import tempfile
import time
import zipfile
from dataclasses import dataclass
from pathlib import Path
from typing import Callable

if __package__ in (None, ""):
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from tools.run_speed_candidates import expand_candidate_patterns, format_seconds, should_stop_for_target


CommandFactory = Callable[[Path, Path], list[str]]


@dataclass(frozen=True)
class RaceResult:
    candidate: Path
    passed: bool
    timed_out: bool
    completion_seconds: float
    target_met: bool
    return_code: int | None
    stdout: str
    stderr: str


def run_candidate(
    candidate: Path,
    runtime_dir: Path,
    command_factory: CommandFactory,
    target_seconds: float,
    max_seconds: float,
) -> RaceResult:
    runtime_dir.mkdir(parents=True, exist_ok=True)
    shutil.copy2(candidate, runtime_dir / "my_car.py")

    started = time.perf_counter()
    try:
        completed = subprocess.run(
            command_factory(candidate, runtime_dir),
            cwd=runtime_dir,
            text=True,
            capture_output=True,
            timeout=max_seconds,
            check=False,
        )
        elapsed = time.perf_counter() - started
        passed = completed.returncode == 0
        return RaceResult(
            candidate=candidate,
            passed=passed,
            timed_out=False,
            completion_seconds=elapsed,
            target_met=should_stop_for_target(passed, elapsed, target_seconds),
            return_code=completed.returncode,
            stdout=completed.stdout,
            stderr=completed.stderr,
        )
    except subprocess.TimeoutExpired as error:
        elapsed = time.perf_counter() - started
        return RaceResult(
            candidate=candidate,
            passed=False,
            timed_out=True,
            completion_seconds=elapsed,
            target_met=False,
            return_code=None,
            stdout=(error.stdout or ""),
            stderr=(error.stderr or ""),
        )


def run_until_target(
    candidates: list[Path],
    runtime_dir: Path,
    command_factory: CommandFactory,
    target_seconds: float,
    max_seconds: float,
) -> list[RaceResult]:
    results: list[RaceResult] = []
    for candidate in candidates:
        result = run_candidate(
            candidate=candidate,
            runtime_dir=runtime_dir,
            command_factory=command_factory,
            target_seconds=target_seconds,
            max_seconds=max_seconds,
        )
        results.append(result)
        if result.target_met:
            break
    return results


def ensure_python_runtime(template_zip: Path, runtime_dir: Path) -> None:
    marker = runtime_dir / "DrivingInterface" / "drive_controller.py"
    if marker.exists():
        return

    runtime_dir.mkdir(parents=True, exist_ok=True)
    prefix = "Template_Python/Bot_Python/"
    with zipfile.ZipFile(template_zip) as archive:
        for entry in archive.infolist():
            if not entry.filename.startswith(prefix) or entry.is_dir():
                continue
            relative = Path(entry.filename[len(prefix):])
            if not relative.parts:
                continue
            target = runtime_dir / relative
            target.parent.mkdir(parents=True, exist_ok=True)
            with archive.open(entry) as source, target.open("wb") as destination:
                shutil.copyfileobj(source, destination)


def copy_settings(settings_file: Path, airsim_settings: Path) -> None:
    airsim_settings.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(settings_file, airsim_settings)


def start_simulator(simulator_dir: Path) -> subprocess.Popen:
    executable = simulator_dir / "Algo.exe"
    return subprocess.Popen(
        [str(executable), "-ResX=640", "-ResY=480", "-windowed"],
        cwd=simulator_dir,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    )


def print_results(results: list[RaceResult], target_seconds: float) -> None:
    print(f"target <= {format_seconds(target_seconds)}")
    print(f"{'candidate':36} {'ok':>3} {'time':>9} {'target':>6} {'timeout':>7}")
    print("-" * 68)
    for result in results:
        print(
            f"{result.candidate.as_posix():36} "
            f"{'yes' if result.passed else 'no':>3} "
            f"{format_seconds(result.completion_seconds):>9} "
            f"{'yes' if result.target_met else 'no':>6} "
            f"{'yes' if result.timed_out else 'no':>7}"
        )


def parse_args() -> argparse.Namespace:
    user_home = Path.home()
    parser = argparse.ArgumentParser(description="Run speed-map candidates until one finishes under a target time.")
    parser.add_argument("candidates", nargs="*", default=["python/my_car.py", "python/variants/*.py"])
    parser.add_argument("--target-seconds", type=float, default=120.0)
    parser.add_argument("--max-seconds", type=float, default=210.0)
    parser.add_argument("--runtime-dir", default=r"C:\SSAFY_RACE\MyCar_Python\Bot_Python")
    parser.add_argument("--template-zip", default=str(user_home / "Downloads" / "MyCar_20260515.zip"))
    parser.add_argument("--settings-file", default="settings/settings-speed.json")
    parser.add_argument("--airsim-settings", default=str(user_home / "Documents" / "AirSim" / "settings.json"))
    parser.add_argument("--simulator-dir", default=r"C:\SSAFY_RACE\Simulator")
    parser.add_argument("--python", default=sys.executable)
    parser.add_argument("--start-simulator", action="store_true")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    candidates = expand_candidate_patterns(args.candidates)
    if not candidates:
        print("No candidates found.", file=sys.stderr)
        return 2

    runtime_dir = Path(args.runtime_dir)
    ensure_python_runtime(Path(args.template_zip), runtime_dir)
    copy_settings(Path(args.settings_file), Path(args.airsim_settings))

    simulator_process = None
    if args.start_simulator:
        simulator_process = start_simulator(Path(args.simulator_dir))
        time.sleep(8.0)

    def command_factory(_candidate: Path, _runtime_dir: Path) -> list[str]:
        return [args.python, "my_car.py"]

    try:
        results = run_until_target(
            candidates=candidates,
            runtime_dir=runtime_dir,
            command_factory=command_factory,
            target_seconds=args.target_seconds,
            max_seconds=args.max_seconds,
        )
    finally:
        if simulator_process is not None and simulator_process.poll() is None:
            simulator_process.terminate()

    print_results(results, args.target_seconds)
    return 0 if results and results[-1].target_met else 1


if __name__ == "__main__":
    raise SystemExit(main())
