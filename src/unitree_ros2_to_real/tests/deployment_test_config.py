"""Resolve the same local deployment profile used by terminal and web launch."""
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[5]
sys.path.insert(0, str(ROOT))
from tools.environment.deployment import render


def mission_config_path():
    return render(ROOT)
