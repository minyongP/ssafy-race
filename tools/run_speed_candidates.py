from __future__ import annotations

import argparse
import concurrent.futures
import os
import shutil
import subprocess
import sys
import tempfile
import time
from dataclasses import dataclass
from pathlib import Path


@dataclass(frozen=True)
class CandidateResult:
    candidate: Path
    passed: bool
    seconds: float
    stdout: str
    stderr: str


def format_seconds(seconds: float) -> str:
    minutes = int(seconds // 60)
    remaining_seconds = seconds - (minutes * 60)
    return f"{minutes:02d}:{remaining_seconds:05.2f}"


def should_stop_for_target(
    passed: bool,
    completion_seconds: float | None,
    target_seconds: float,
) -> bool:
    return passed and completion_seconds is not None and completion_seconds <= target_seconds


def default_jobs() -> int:
    logical_cpus = os.cpu_count() or 1
    return max(1, min(8, logical_cpus // 2))


def expand_candidate_patterns(patterns: list[str]) -> list[Path]:
    candidates: list[Path] = []
    for pattern in patterns:
        matches = sorted(Path().glob(pattern))
        if matches:
            candidates.extend(matches)
            continue
        path = Path(pattern)
        if path.exists():
            candidates.append(path)

    unique: dict[Path, Path] = {}
    for candidate in candidates:
        resolved = candidate.resolve()
        unique[resolved] = candidate
    return list(unique.values())


def run_candidate(candidate: Path, test_file: Path, python_executable: str) -> CandidateResult:
    started = time.perf_counter()
    with tempfile.TemporaryDirectory(prefix="ssafy-speed-") as tmp:
        tmp_path = Path(tmp)
        shutil.copy2(candidate, tmp_path / "my_car.py")
        shutil.copy2(test_file, tmp_path / "test_my_car_logic.py")

        completed = subprocess.run(
            [
                python_executable,
                "-m",
                "unittest",
                "discover",
                "-s",
                str(tmp_path),
                "-v",
            ],
            text=True,
            capture_output=True,
            check=False,
        )

    return CandidateResult(
        candidate=candidate,
        passed=completed.returncode == 0,
        seconds=time.perf_counter() - started,
        stdout=completed.stdout,
        stderr=completed.stderr,
    )


def run_candidates(
    candidates: list[Path],
    test_file: Path,
    jobs: int,
    python_executable: str,
) -> list[CandidateResult]:
    with concurrent.futures.ThreadPoolExecutor(max_workers=max(1, jobs)) as executor:
        futures = [
            executor.submit(run_candidate, candidate, test_file, python_executable)
            for candidate in candidates
        ]
        results = [future.result() for future in concurrent.futures.as_completed(futures)]

    return sorted(results, key=lambda result: (not result.passed, result.seconds, result.candidate.name))


def print_results(results: list[CandidateResult]) -> None:
    print(f"{'candidate':36} {'ok':>3} {'ms':>10}")
    print("-" * 53)
    for result in results:
        print(f"{result.candidate.as_posix():36} {'yes' if result.passed else 'no':>3} {result.seconds * 1000:10.2f}")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Run SSAFY Race Python speed candidates in parallel.")
    parser.add_argument(
        "candidates",
        nargs="*",
        default=["python/my_car.py", "python/variants/*.py"],
        help="Candidate files or glob patterns. Each candidate is tested as my_car.py.",
    )
    parser.add_argument("--test-file", default="python/test_my_car_logic.py")
    parser.add_argument("--jobs", type=int, default=default_jobs())
    parser.add_argument("--python", default=sys.executable)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    candidates = expand_candidate_patterns(args.candidates)
    test_file = Path(args.test_file)

    if not candidates:
        print("No candidates found.", file=sys.stderr)
        return 2
    if not test_file.exists():
        print(f"Test file not found: {test_file}", file=sys.stderr)
        return 2

    results = run_candidates(candidates, test_file, args.jobs, args.python)
    print_results(results)

    failed = [result for result in results if not result.passed]
    if failed:
        print("\nFailures:", file=sys.stderr)
        for result in failed:
            print(f"\n== {result.candidate} ==", file=sys.stderr)
            print(result.stdout, file=sys.stderr)
            print(result.stderr, file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
