import sys
import tempfile
import textwrap
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from tools.run_speed_candidates import format_seconds, should_stop_for_target, run_candidates


class SpeedParallelRunnerTests(unittest.TestCase):
    def test_should_stop_when_completed_under_target_seconds(self):
        self.assertTrue(should_stop_for_target(passed=True, completion_seconds=119.5, target_seconds=120.0))
        self.assertTrue(should_stop_for_target(passed=True, completion_seconds=120.0, target_seconds=120.0))
        self.assertFalse(should_stop_for_target(passed=True, completion_seconds=121.0, target_seconds=120.0))
        self.assertFalse(should_stop_for_target(passed=False, completion_seconds=90.0, target_seconds=120.0))
        self.assertFalse(should_stop_for_target(passed=True, completion_seconds=None, target_seconds=120.0))

    def test_format_seconds_for_two_minute_threshold_output(self):
        self.assertEqual(format_seconds(119.5), "01:59.50")
        self.assertEqual(format_seconds(120.0), "02:00.00")

    def test_run_candidates_orders_passing_candidates_first(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            tests_dir = root / "tests"
            tests_dir.mkdir()
            test_file = tests_dir / "test_my_car_logic.py"
            test_file.write_text(
                textwrap.dedent(
                    """
                    import unittest
                    from my_car import value

                    class CandidateTest(unittest.TestCase):
                        def test_value(self):
                            self.assertEqual(value(), 1)
                    """
                ).strip()
                + "\n",
                encoding="utf-8",
            )

            passing = root / "passing.py"
            passing.write_text("def value():\n    return 1\n", encoding="utf-8")
            failing = root / "failing.py"
            failing.write_text("def value():\n    return 2\n", encoding="utf-8")

            results = run_candidates(
                candidates=[failing, passing],
                test_file=test_file,
                jobs=2,
                python_executable=sys.executable,
            )

        self.assertEqual([result.candidate.name for result in results], ["passing.py", "failing.py"])
        self.assertTrue(results[0].passed)
        self.assertFalse(results[1].passed)


if __name__ == "__main__":
    unittest.main()
