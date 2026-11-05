import copy
import sys
import unittest
from pathlib import Path
from unittest import mock

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import reuse_published_binary as reuse

class PublishedBinaryReuse(unittest.TestCase):
    def test_rejects_changed_native_source(self):
        def git(*args):
            if args[0] == 'rev-parse': return reuse.TAG_COMMIT
            if args[0] == 'diff': return 'adapters/wpilib-2026/src/LoadedMotorIO.cpp'
            raise AssertionError(args)
        with mock.patch.object(reuse, 'output', side_effect=git):
            with self.assertRaises(ValueError): reuse.verify_source()

    def test_provenance_checks_original_and_archive_identity(self):
        source = reuse.manifest()
        record = {'buildMode':'reused-published-binary', 'binarySourceTag':reuse.TAG,
                  'binaryTagCommit':reuse.TAG_COMMIT, 'binarySourceCommit':reuse.SOURCE_COMMIT,
                  'binaryManifestSha256':reuse.MANIFEST_SHA, 'binaryArchiveSha256':reuse.ARCHIVES['linuxx86-64'],
                  'originalBuildProvenance':source['buildProvenance']['linuxx86-64']}
        with mock.patch.object(reuse, 'verify_source'):
            reuse.verify_provenance('linuxx86-64', record)
            altered = copy.deepcopy(record)
            altered['binaryArchiveSha256']['Release'] = '0' * 64
            with self.assertRaises(ValueError): reuse.verify_provenance('linuxx86-64', altered)
            altered = copy.deepcopy(record)
            altered['originalBuildProvenance']['baseCommit'] = '0' * 40
            with self.assertRaises(ValueError): reuse.verify_provenance('linuxx86-64', altered)
