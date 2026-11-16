"""Where the repo's data/ folder (block world, cached plans) is: works from the source tree and from the installed package."""
import os
from pathlib import Path


def data_dir() -> Path:
    candidates = []
    if os.environ.get("GR_BLOCKS_DATA"):
        candidates.append(Path(os.environ["GR_BLOCKS_DATA"]))
    candidates.append(Path(__file__).resolve().parents[2] / "data")  # source tree: <repo>/gr_blocks_ros/gr_blocks_ros/ -> <repo>/data
    try:
        from ament_index_python.packages import get_package_share_directory

        candidates.append(Path(get_package_share_directory("gr_blocks_ros")) / "data")
    except Exception:  # noqa: BLE001  (not installed / no ament index)
        pass
    for c in candidates:
        if (c / "blockworld.json").exists():
            return c
    raise FileNotFoundError(f"data/ folder not found, looked in {[str(c) for c in candidates]}")
