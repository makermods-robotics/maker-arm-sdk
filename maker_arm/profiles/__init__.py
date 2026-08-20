"""Versioned, read-only hardware model profiles shipped with maker-arm."""

from pathlib import Path


PROFILE_DIR = Path(__file__).resolve().parent
DEFAULT_ARM_CONFIG = PROFILE_DIR / "maker_arm_v1.yaml"
DEFAULT_STAR_MAPPING = PROFILE_DIR / "star_to_maker_v1.json"

__all__ = ["DEFAULT_ARM_CONFIG", "DEFAULT_STAR_MAPPING", "PROFILE_DIR"]
