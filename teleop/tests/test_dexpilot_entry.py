"""Short DexPilot entry and controller handshake; no camera, ROS or CAN."""
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import patch

ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'teleop/bridge'))
import lh20_cli
from live_hand import HandController
import test_factory_hand as factory_tests


class DexPilotEntryTests(unittest.TestCase):
    def test_short_entry_keeps_normal_mode_and_routes_dexpilot(self):
        with tempfile.TemporaryDirectory() as folder:
            path=Path(folder)
            entry=path/'lh20-left';entry.write_bytes((ROOT/'lh20-left').read_bytes());entry.chmod(0o755)
            stub=path/'lh20';stub.write_text('#!/bin/bash\nprintf "%s\\n" "$@"\n');stub.chmod(0o755)
            normal=subprocess.check_output([str(entry),'vision'],text=True).splitlines()
            self.assertNotIn('dexpilot',normal)
            command=subprocess.check_output([str(entry),'vision','dexpilot','--preview'],text=True).splitlines()
            self.assertEqual(command[command.index('--retargeting')+1],'dexpilot')
            self.assertIn('--preview',command)
            self.assertEqual(command[command.index('--thumb-axes')+1],'swapped')
            self.assertEqual(command[command.index('--scope')+1],'hand')
            for alias in ('vector',):
                command=subprocess.check_output([str(entry),'vision',alias,'--preview'],text=True).splitlines()
                self.assertEqual(command[command.index('--retargeting')+1],'vector')
                self.assertNotIn(alias,command[:command.index('--retargeting')])
                self.assertIn('--preview',command)

    def test_algorithm_routes_to_both_automatic_and_camera(self):
        for extra,method in (([],'execv'),(['--preview'],'execve'),(['--self-test'],'execve')):
            with patch.object(lh20_cli.os,method,side_effect=SystemExit(0)) as run:
                with self.assertRaises(SystemExit):
                    lh20_cli.main(['vision','--hand','left','--scope','hand','--retargeting','dexpilot',*extra])
                command=run.call_args.args[1]
                self.assertEqual(command[command.index('--retargeting')+1],'dexpilot')
        self.assertEqual(lh20_cli.main(['vision','--hand','left','--retargeting','dexpilot']),1)
        self.assertEqual(lh20_cli.main(['vision','--hand','left','--scope','hand','--retargeting','dexpilot','--thumb-profile','palm']),1)

    def test_controller_accepts_matching_algorithm_and_rejects_mixed_stream(self):
        helper=factory_tests.FactoryHandTests();helper.setUp()
        helper.engine=HandController(helper.baseline,helper.m,algorithm='dexpilot')
        for _ in range(3):helper.feed(retargeting_algorithm='dexpilot')
        self.assertTrue(helper.engine.active)
        self.assertIsNone(helper.feed(retargeting_algorithm='vector'))
        self.assertIn('Wrong retargeting algorithm',helper.engine.state)
        for _ in range(3):helper.feed(retargeting_algorithm='dexpilot',retargeting_config_sha256='test-config')
        self.assertTrue(helper.engine.active)
        self.assertEqual(helper.engine.retargeting_config_sha256,'test-config')


if __name__=='__main__':unittest.main()
