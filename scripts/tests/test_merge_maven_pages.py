import hashlib
import json
from pathlib import Path
import sys
import tempfile
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from merge_maven_pages import merge_pages


class MergeMavenPagesTest(unittest.TestCase):
    def test_preserves_old_artifacts_and_updates_current_descriptor(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            new_root = root / "new"
            (new_root / "maven/new-version").mkdir(parents=True)
            new_bytes = b"new artifact"
            (new_root / "maven/new-version/package.zip").write_bytes(new_bytes)
            new_manifest = {"version": "new", "artifacts": {
                "maven/new-version/package.zip": hashlib.sha256(new_bytes).hexdigest()
            }}
            (new_root / "SHA256SUMS.json").write_text(json.dumps(new_manifest))
            (new_root / "MecoSimulation.json").write_text('{"version":"new"}')
            old_bytes = b"old artifact"
            old_digest = hashlib.sha256(old_bytes).hexdigest()
            old_manifest = json.dumps({"artifacts": {"maven/old-version/package.zip": old_digest}}).encode()
            old_files = {"SHA256SUMS.json": old_manifest, "maven/old-version/package.zip": old_bytes}
            out = root / "pages"

            merge_pages(new_root, out, old_files.get)

            self.assertEqual((out / "maven/old-version/package.zip").read_bytes(), old_bytes)
            merged = json.loads((out / "SHA256SUMS.json").read_text())
            self.assertEqual(set(merged["artifacts"]), {
                "maven/new-version/package.zip", "maven/old-version/package.zip"
            })
            self.assertEqual(json.loads((out / "MecoSimulation.json").read_text())["version"], "new")

    def test_refuses_to_replace_an_existing_coordinate_with_different_bytes(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            new_root = root / "new"
            (new_root / "maven/version").mkdir(parents=True)
            (new_root / "maven/version/package.zip").write_bytes(b"new")
            (new_root / "SHA256SUMS.json").write_text(json.dumps({"artifacts": {
                "maven/version/package.zip": hashlib.sha256(b"new").hexdigest()
            }}))
            old_manifest = json.dumps({"artifacts": {
                "maven/version/package.zip": hashlib.sha256(b"old").hexdigest()
            }}).encode()
            with self.assertRaisesRegex(ValueError, "immutable Maven artifact"):
                merge_pages(new_root, root / "pages", lambda path: old_manifest if path == "SHA256SUMS.json" else b"old")


if __name__ == "__main__":
    unittest.main()
