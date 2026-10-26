from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from write_build_provenance import checkout_state


class BuildProvenanceTest(unittest.TestCase):
    def test_source_changes_are_dirty_but_generated_output_is_excluded(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            def git(*args):
                subprocess.run(['git', *args], cwd=root, check=True, capture_output=True)
            git('init')
            source = root / 'plant.cpp'
            source.write_text('original\n')
            git('add', 'plant.cpp')
            git('-c', 'user.name=Test', '-c', 'user.email=test@example.com',
                'commit', '-m', 'fixture')
            clean, original_hash = checkout_state(root, ['build'])
            self.assertTrue(clean)
            (root / 'build').mkdir()
            (root / 'build/output').write_text('generated')
            self.assertEqual(checkout_state(root, ['build']), (True, original_hash))
            source.write_text('modified\n')
            dirty, changed_hash = checkout_state(root, ['build'])
            self.assertFalse(dirty)
            self.assertNotEqual(changed_hash, original_hash)
            source.write_text('original\n')
            (root / 'untracked.h').write_text('untracked source')
            self.assertFalse(checkout_state(root, ['build'])[0])
