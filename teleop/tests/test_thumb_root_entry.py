"""Thumb root mode routing only; no ROS, camera or device access."""
from pathlib import Path
import sys
import unittest
from unittest.mock import patch

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'bridge'))
import lh20_cli


class ThumbRootEntryTests(unittest.TestCase):
    def test_modes_and_default_reach_camera_and_automatic_session(self):
        for mode in ('agile','directions',None):
            option=['--thumb-root-mode',mode] if mode else []
            for extra,method in (([],'execv'),(['--preview'],'execve'),(['--self-test'],'execve')):
                with patch.object(lh20_cli.os,method,side_effect=SystemExit(0)) as run:
                    with self.assertRaises(SystemExit):
                        lh20_cli.main(['vision','--hand','left','--scope','hand',*option,*extra])
                    command=run.call_args.args[1]
                    self.assertEqual(command[command.index('--thumb-root-mode')+1],mode or 'agile')


if __name__=='__main__':unittest.main()
