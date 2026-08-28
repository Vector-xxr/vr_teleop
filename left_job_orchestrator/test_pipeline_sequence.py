from __future__ import annotations

import tempfile
import unittest
from pathlib import Path

from left_job_orchestrator.pipeline import preflight_job


class PipelineSequenceTest(unittest.TestCase):
    def make_job(self, *names: str) -> Path:
        temp = tempfile.TemporaryDirectory()
        self.addCleanup(temp.cleanup)
        root = Path(temp.name)
        for name in names:
            (root / name).touch()
        return root

    def labels(self, *names: str) -> list[str]:
        return [step.label for step in preflight_job(self.make_job(*names))]

    def test_single_action_is_valid(self) -> None:
        self.assertEqual(self.labels("action1.csv"), ["action1"])

    def test_numbered_steps_define_order(self) -> None:
        self.assertEqual(
            self.labels(
                "action4.csv",
                "visual_grasp2.txt",
                "action1.csv",
                "action3.csv",
            ),
            ["action1", "visual_grasp2", "action3", "action4"],
        )

    def test_legacy_visual_is_between_action1_and_action2(self) -> None:
        self.assertEqual(
            self.labels("action2.csv", "visual_grasp.txt", "action1.csv"),
            ["action1", "visual_grasp", "action2"],
        )

    def test_grisp_alias_is_accepted(self) -> None:
        self.assertEqual(
            self.labels("action1.csv", "visual_grisp2.txt", "action3.csv"),
            ["action1", "visual_grisp2", "action3"],
        )

    def test_visual_with_same_number_runs_before_action(self) -> None:
        self.assertEqual(
            self.labels("action1.csv", "action2.csv", "visual_grasp2.txt"),
            ["action1", "visual_grasp2", "action2"],
        )

    def test_duplicate_visual_step_is_rejected(self) -> None:
        with self.assertRaisesRegex(ValueError, "duplicate visual_grasp step 2"):
            preflight_job(
                self.make_job(
                    "action1.csv",
                    "visual_grasp2.txt",
                    "visual_grisp2.txt",
                    "action3.csv",
                )
            )

    def test_visual_steps_cannot_be_consecutive(self) -> None:
        with self.assertRaisesRegex(ValueError, "must be followed"):
            preflight_job(
                self.make_job(
                    "action1.csv",
                    "visual_grasp2.txt",
                    "visual_grasp3.txt",
                    "action4.csv",
                )
            )


if __name__ == "__main__":
    unittest.main()
