import json
from pathlib import Path
import sys
import tempfile
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from verify_release_tag import validate


class VerifyReleaseTagTest(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.descriptor = Path(self.temporary.name) / "MecoSimulation.json"
        self.descriptor.write_text(json.dumps({"version": "2026.0.3"}), encoding="utf-8")

    def tearDown(self):
        self.temporary.cleanup()

    def test_accepts_matching_tag(self):
        validate("refs/tags/v2026.0.3", self.descriptor)

    def test_rejects_tag_for_different_package_version(self):
        with self.assertRaisesRegex(ValueError, "does not match vendordep version"):
            validate("refs/tags/v2026.0.2", self.descriptor)

    def test_branch_build_is_not_a_release_tag(self):
        validate("refs/heads/codex/validation", self.descriptor)


if __name__ == "__main__":
    unittest.main()
