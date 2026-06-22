import subprocess
import sys
import tempfile
import textwrap
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from tools.run_speed_race_until_target import run_until_target


class SpeedRaceUntilTargetTests(unittest.TestCase):
    def test_stops_after_first_candidate_that_finishes_within_two_minutes(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            candidates = []
            for name, sleep_seconds, exit_code in [
                ("slow.py", 0.20, 0),
                ("fast.py", 0.01, 0),
                ("unused.py", 0.01, 0),
            ]:
                candidate = root / name
                candidate.write_text("# candidate placeholder\n", encoding="utf-8")
                candidates.append(candidate)
                runner = root / f"{name}.runner.py"
                runner.write_text(
                    f"import time, sys\ntime.sleep({sleep_seconds})\nsys.exit({exit_code})\n",
                    encoding="utf-8",
                )

            def command_factory(candidate, _runtime_dir):
                return [sys.executable, str(root / f"{candidate.name}.runner.py")]

            results = run_until_target(
                candidates=candidates,
                runtime_dir=root / "runtime",
                command_factory=command_factory,
                target_seconds=0.15,
                max_seconds=1.0,
            )

        self.assertEqual([result.candidate.name for result in results], ["slow.py", "fast.py"])
        self.assertFalse(results[0].target_met)
        self.assertTrue(results[1].target_met)

    def test_timeout_candidate_does_not_meet_target(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            candidate = root / "timeout.py"
            candidate.write_text("# candidate placeholder\n", encoding="utf-8")
            runner = root / "timeout.py.runner.py"
            runner.write_text("import time\ntime.sleep(0.2)\n", encoding="utf-8")

            def command_factory(_candidate, _runtime_dir):
                return [sys.executable, str(runner)]

            results = run_until_target(
                candidates=[candidate],
                runtime_dir=root / "runtime",
                command_factory=command_factory,
                target_seconds=0.03,
                max_seconds=0.05,
            )

        self.assertEqual(len(results), 1)
        self.assertFalse(results[0].passed)
        self.assertTrue(results[0].timed_out)
        self.assertFalse(results[0].target_met)


if __name__ == "__main__":
    unittest.main()
