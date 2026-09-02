from __future__ import annotations

from copy import deepcopy
from pathlib import Path
from typing import Any

import yaml


def load_config(path: str | Path, profile: str) -> dict[str, Any]:
    path = Path(path).resolve()
    with path.open("r", encoding="utf-8") as handle:
        config = yaml.safe_load(handle)
    if profile not in config["profiles"]:
        raise KeyError(f"Unknown profile {profile!r}; choose from {sorted(config['profiles'])}")
    merged = deepcopy(config)
    merged["profile_name"] = profile
    merged["profile"] = deepcopy(config["profiles"][profile])
    merged["config_path"] = str(path)
    return merged
