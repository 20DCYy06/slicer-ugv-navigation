#!/usr/bin/env python3
"""Inspect the offline container command with fake Docker; no container starts."""
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

SCRIPT = Path(__file__).with_name('noetic.sh')


class OfflineShellTests(unittest.TestCase):
    def run_command(self, reference, *command):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            captured = root / 'args.json'
            docker = root / 'docker'
            docker.write_text('#!' + sys.executable + '\n'
                              'import json,os,sys\n'
                              'open(os.environ["UGV_TEST_ARGS"],"w").write(json.dumps(sys.argv[1:]))\n')
            docker.chmod(0o755)
            env = dict(os.environ, PATH=str(root) + os.pathsep + os.environ['PATH'],
                       UGV_TEST_ARGS=str(captured), UGV_SLICER_ROOT=reference,
                       UGV_NOETIC_IMAGE='offline-test:no-run')
            result = subprocess.run(['bash', str(SCRIPT), *command], env=env,
                                    stdin=subprocess.DEVNULL, capture_output=True,
                                    text=True, timeout=10)
            self.assertEqual(result.returncode, 0, result.stderr)
            return json.loads(captured.read_text())

    def test_reference_sources_are_available_read_only_with_network_and_devices_absent(self):
        with tempfile.TemporaryDirectory(prefix='source tree ') as ref:
            (Path(ref) / 'catkin_ws/src').mkdir(parents=True)
            args = self.run_command(ref, 'bash', '-c', 'printf harmless')
        self.assertIn('type=bind,src=' + ref + ',dst=/slicer,readonly', args)
        self.assertEqual(args[args.index('--network') + 1], 'none')
        self.assertNotIn('--privileged', args)
        self.assertFalse(any(x.startswith('--device') for x in args))
        self.assertEqual(args[-3:], ['bash', '-c', 'printf harmless'])

    def test_without_reference_existing_offline_shell_still_works(self):
        args = self.run_command('/definitely/absent/offline-reference', 'python3', '-V')
        self.assertFalse(any('dst=/slicer' in x for x in args))
        self.assertEqual(args[args.index('--network') + 1], 'none')
        self.assertEqual(args[-2:], ['python3', '-V'])

    def test_command_arguments_are_not_evaluated_by_wrapper(self):
        command = ('printf', '%s', '$(must-not-execute)', '', 'two words')
        args = self.run_command('/definitely/absent/offline-reference', *command)
        self.assertEqual(args[-len(command):], list(command))


if __name__ == '__main__':
    unittest.main(verbosity=2)
