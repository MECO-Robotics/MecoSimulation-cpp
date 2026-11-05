import copy
import sys
import json
import subprocess
from pathlib import Path
import unittest
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
sys.path.insert(0, str(Path(__file__).resolve().parents[3] / 'scripts'))
from validate_tested_release import REQUIRED_JOBS, validate
from reuse_simulation_core import sha256
from validate_job_deadlines import validate_paired_consumers


class TestedReleaseTest(unittest.TestCase):
    def setUp(self):
        self.run = {'event': 'pull_request', 'status': 'completed', 'conclusion': 'success', 'head_sha': 'head'}
        self.jobs = [{'name': name, 'status': 'completed', 'conclusion': 'success',
                      'created_at': '2026-11-05T12:00:00Z', 'completed_at': '2026-11-05T12:03:00Z'}
                     for name in REQUIRED_JOBS]
        self.commit = {'sha': 'built', 'parents': [{'sha': 'base'}, {'sha': 'head'}], 'tree': {'sha': 'tree'}}
        self.tag = {'parents': [{'sha': 'base'}, {'sha': 'head'}], 'tree': {'sha': 'tree'}}
        pinned = json.loads((Path(__file__).resolve().parents[3] / 'scripts/fast_sim_core.json').read_text())
        members = {'Core.o': '0' * 64}
        member_sha = sha256(json.dumps(members, sort_keys=True, separators=(',', ':')).encode())
        retained = {}
        for platform in ('linuxx86-64', 'windowsx86-64'):
            retained[platform] = {}
            for config in ('Debug', 'Release'):
                retained[platform][config] = {
                    'platform': platform,
                    'configuration': config,
                    'sourceTag': pinned['sourceTag'],
                    'sourceCommit': pinned['sourceCommit'],
                    'baseVendordepVersion': pinned['baseVendordepVersion'],
                    'baseCommit': pinned['baseCommit'],
                    'manifestSha256': pinned['manifestSha256'],
                    'packageArchiveSha256': pinned['archives'][platform][config],
                    'coreSourceTrees': pinned['coreSourceTrees'],
                    'retainedMemberCount': 1,
                    'retainedMembers': members,
                    'retainedMembersSha256': member_sha,
                }
        self.item = {'simulationCommit': 'built', 'simulationWorkingTreeClean': True,
                     'baseWorkingTreeClean': True, 'baseVendordepVersion': '2026.2.12', 'baseCommit': 'pinned',
                     'buildMode': 'reused-pinned-core', 'coreBuild': None,
                     'retainedCore': retained['linuxx86-64']}
        self.manifest = {'sourceCommit': 'built', 'version': '2026.0.17',
                         'buildProvenance': {'linuxx86-64': self.item,
                                             'windowsx86-64': {**self.item, 'retainedCore': retained['windowsx86-64']}}}
        self.descriptor = {'version': '2026.0.17'}

    def check(self):
        validate(self.run, self.jobs, self.manifest, self.tag, self.commit,
                 self.descriptor, self.descriptor, 'pinned')

    def test_accepts_same_tree_and_completed_checks(self):
        self.check()

    def test_accepts_full_core_build_when_tree_and_test_evidence_match(self):
        self.manifest = copy.deepcopy(self.manifest)
        root = Path(__file__).resolve().parents[3]
        trees = {path: subprocess.check_output(['git', 'rev-parse', f'HEAD:{path}'], cwd=root, text=True).strip()
                 for path in ('CMakeLists.txt', 'include', 'src', 'tests')}
        for item in self.manifest['buildProvenance'].values():
            item['buildMode'] = 'full-source-build'
            item['retainedCore'] = None
            item['coreBuild'] = {'mode': 'full-source-build', 'sourceTrees': trees,
                                 'releaseTestsPassed': True, 'debugBuildPassed': True}
        self.check()

    def test_rejects_different_source_tree_or_missing_consumer(self):
        self.commit['tree'] = {'sha': 'other'}
        with self.assertRaises(ValueError): self.check()
        self.commit['tree'] = {'sha': 'tree'}
        self.jobs.pop()
        with self.assertRaises(ValueError): self.check()

    def test_requires_queue_inclusive_deadline_and_timestamps(self):
        self.jobs[0]['completed_at'] = '2026-11-05T12:03:01Z'
        with self.assertRaises(ValueError): self.check()
        self.jobs[0]['completed_at'] = None
        with self.assertRaises(ValueError): self.check()
        self.jobs[0]['completed_at'] = '2026-11-05T11:59:59Z'
        with self.assertRaises(ValueError): self.check()

    def test_paired_consumer_matrix_requires_all_versions_within_deadline(self):
        jobs = [dict(name=f'{group} ({platform}, {config}, v{version}, {version})',
                     status='completed', conclusion='success',
                     created_at='2026-11-05T12:00:00Z', completed_at='2026-11-05T12:03:00Z')
                for group in ('deployed-warm-dependencies', 'deployed-consumer')
                for platform in ('ubuntu-24.04', 'windows-2022')
                for config in ('Debug', 'Release')
                for version in ('2026.2.12', '2026.2.15')]
        validate_paired_consumers(jobs)
        with self.assertRaises(ValueError): validate_paired_consumers(jobs[:-1])
        jobs[-1]['completed_at'] = '2026-11-05T12:03:01Z'
        with self.assertRaises(ValueError): validate_paired_consumers(jobs)

    def test_rejects_dirty_or_unpinned_platform(self):
        self.manifest = copy.deepcopy(self.manifest)
        self.manifest['buildProvenance']['windowsx86-64']['baseWorkingTreeClean'] = False
        with self.assertRaises(ValueError): self.check()
        self.manifest['buildProvenance']['windowsx86-64']['baseWorkingTreeClean'] = True
        self.manifest['buildProvenance']['windowsx86-64']['baseCommit'] = 'other'
        with self.assertRaises(ValueError): self.check()


if __name__ == '__main__': unittest.main()
