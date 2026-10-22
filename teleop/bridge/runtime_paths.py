"""Find the isolated vision interpreter without depending on a Linux username."""
import os
from pathlib import Path
import subprocess


def vision_python():
    explicit = os.environ.get('LH20_VISION_PYTHON')
    if explicit:
        return Path(explicit).expanduser().resolve()
    base = os.environ.get('LH20_CONDA_BASE')
    if not base:
        try:
            base = subprocess.check_output(['conda', 'info', '--base'], text=True, timeout=10).strip()
        except (OSError, subprocess.SubprocessError):
            base = None
    candidates = [Path(base)] if base else [Path.home()/n for n in ('miniconda3','miniforge3','anaconda3')]
    for root in candidates:
        path = root/'envs/lh20_vision/bin/python'
        if path.is_file():
            return path
    raise RuntimeError('找不到 lh20_vision；请设置 LH20_VISION_PYTHON 或 LH20_CONDA_BASE')
