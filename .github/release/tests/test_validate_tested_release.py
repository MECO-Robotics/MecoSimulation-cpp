import copy
import sys
from pathlib import Path
import unittest
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from validate_tested_release import REQUIRED_JOBS, validate


class TestedReleaseTest(unittest.TestCase):
    def setUp(self):
        self.run = {'event': 'pull_request', 'status': 'completed', 'conclusion': 'success', 'head_sha': 'head'}
        self.jobs = [{'name': name, 'status': 'completed', 'conclusion': 'success'} for name in REQUIRED_JOBS]
        self.commit = {'sha': 'built', 'parents': [{'sha': 'base'}, {'sha': 'head'}], 'tree': {'sha': 'tree'}}
        self.tag = {'parents': [{'sha': 'base'}, {'sha': 'head'}], 'tree': {'sha': 'tree'}}
        self.item = {'simulationCommit': 'built', 'simulationWorkingTreeClean': True,
                     'baseWorkingTreeClean': True, 'baseVendordepVersion': '2026.2.5', 'baseCommit': 'pinned'}
        self.manifest = {'sourceCommit': 'built', 'version': '2026.0.16',
                         'buildProvenance': {'linuxx86-64': self.item, 'windowsx86-64': self.item}}
        self.descriptor = {'version': '2026.0.16'}

    def check(self):
        validate(self.run, self.jobs, self.manifest, self.tag, self.commit,
                 self.descriptor, self.descriptor, 'pinned')

    def test_accepts_same_tree_and_completed_checks(self):
        self.check()

    def test_rejects_different_source_tree_or_missing_consumer(self):
        self.commit['tree'] = {'sha': 'other'}
        with self.assertRaises(ValueError): self.check()
        self.commit['tree'] = {'sha': 'tree'}
        self.jobs.pop()
        with self.assertRaises(ValueError): self.check()

    def test_rejects_dirty_or_unpinned_platform(self):
        self.manifest = copy.deepcopy(self.manifest)
        self.manifest['buildProvenance']['windowsx86-64']['baseWorkingTreeClean'] = False
        with self.assertRaises(ValueError): self.check()
        self.manifest['buildProvenance']['windowsx86-64']['baseWorkingTreeClean'] = True
        self.manifest['buildProvenance']['windowsx86-64']['baseCommit'] = 'other'
        with self.assertRaises(ValueError): self.check()


if __name__ == '__main__': unittest.main()
