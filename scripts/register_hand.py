"""Explicit local device enrollment; the resulting identity is Git-ignored."""
from pathlib import Path
import os
import shlex
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'teleop/bridge'))


def main():
    from auto_vision import read_identity
    from lh20_cli import check_can
    path = ROOT/'.local.env'
    if path.exists():
        raise SystemExit('.local.env 已存在，拒绝覆盖；请先检查已登记的设备。')
    check_can('can0')
    identity = read_identity()
    if not identity.startswith('LHT20-') or '-L-' not in identity:
        raise SystemExit('不是预期的 L20 左手身份，未登记。')
    fd = os.open(path, os.O_WRONLY|os.O_CREAT|os.O_EXCL, 0o600)
    with os.fdopen(fd, 'w') as stream:
        stream.write('LH20_EXPECTED_IDENTIFIER='+shlex.quote(identity)+'\n')
    print('左手身份已登记；设备型号和固件仍由控制桥检查。文件不会提交到 Git。')


if __name__ == '__main__':
    main()
