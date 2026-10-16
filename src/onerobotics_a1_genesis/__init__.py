"""OneRobotics A1 robot assets and Genesis simulation examples."""

from pathlib import Path

__version__ = "0.1.0"


def asset_dir() -> Path:
    """Return the bundled A1 URDF and mesh directory."""
    return Path(__file__).resolve().parent / "assets" / "onerobotics_a1"


def urdf_path(robot: str = "right") -> Path:
    """Return a local URDF path for `right`, `left`, or `bimanual`."""
    rel = {
        "right": "a1_r.urdf",
        "left": "a1_l.urdf",
        "bimanual": "bimanual_stand/a1_bimanual_stand.urdf",
    }
    try:
        path = asset_dir() / rel[robot]
    except KeyError as exc:
        raise ValueError("robot must be right, left, or bimanual") from exc
    if not path.is_file():
        raise FileNotFoundError(f"A1 model asset is missing: {path}")
    return path
