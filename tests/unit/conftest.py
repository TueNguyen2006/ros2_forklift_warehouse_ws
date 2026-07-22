"""Make the in-tree Python package importable for pure-core tests."""

import sys
from pathlib import Path


SOURCE_ROOT = Path(__file__).resolve().parents[2] / "src" / "warehouse_visual_localization"
if str(SOURCE_ROOT) not in sys.path:
    sys.path.insert(0, str(SOURCE_ROOT))
