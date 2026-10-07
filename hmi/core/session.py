"""Job sessions: a job together with the config (variant) it belongs to (docs/HMI_DESIGN.md D-H11, section 8.1).

A job file records its config variant (meta "config_variant", tools/make_job.py); the HMI always loads that
variant, as tools/run_job.py does, and refuses a mismatch. Building from the config runs tools/make_job.py
build_nominal (seconds - the RunController calls it in the run thread).
"""
from __future__ import annotations

import importlib.util
from dataclasses import dataclass
from pathlib import Path

from mauer import REPO
from mauer import config as mconfig
from mauer import job as mjob
from mauer.job import Job

JOBS_DIR = REPO / "data" / "jobs"


class SessionError(ValueError):
    """The job cannot be used with this config (unreadable, invalid, other variant, build failed)."""


@dataclass(frozen=True)
class JobSession:
    cfg: dict                 # mauer.config.load(config_path, variant)
    variant: str | None       # mauer.config.variant_of(cfg)
    job: Job
    path: Path | None         # job file; None = built from the config
    source: str               # "file" | "built" | "test"
    name: str                 # file stem, or "nominal_<shape><config.suffix(cfg)>"

    @property
    def variant_label(self) -> str:
        """The variant name for display: 'main' for config/station.toml alone."""
        return self.variant or "main"


def list_jobs(folder: Path | None = None) -> list[Path]:
    """data/jobs/*.json, newest first."""
    d = Path(folder) if folder else JOBS_DIR
    return sorted(d.glob("*.json"), key=lambda p: p.stat().st_mtime, reverse=True) if d.is_dir() else []


def list_variants() -> list[str]:
    """Names of config/variants/*.toml."""
    return sorted(p.stem for p in mconfig.VARIANTS.glob("*.toml")) if mconfig.VARIANTS.is_dir() else []


def load_job_file(path: str | Path, config_path: str | Path | None = None) -> JobSession:
    """The job file with the config variant it was built with (meta config_variant, as tools/run_job.py)."""
    path = Path(path)
    try:
        variant = mjob.load(path, check=False).meta.get("config_variant") or None
    except (OSError, ValueError, KeyError, TypeError) as e:
        raise SessionError(f"job file {path} not readable: {e}") from e
    try:
        cfg = mconfig.load(config_path, variant)
    except (OSError, ValueError) as e:
        raise SessionError(f"job {path.name} needs config variant {variant!r}: {e}") from e
    if variant != mconfig.variant_of(cfg):
        raise SessionError(f"job {path.name} was built with config variant {variant!r}, the config loaded is "
                           f"{mconfig.variant_of(cfg)!r}")
    try:
        job = mjob.load(path)
    except mjob.JobError as e:
        raise SessionError(f"job {path.name}: {e}") from e
    return JobSession(cfg, variant, job, path, "file", path.stem)


def make_job_tool():
    """tools/make_job.py as a module (loaded by path like tests/test_sequencer.py _tool)."""
    spec = importlib.util.spec_from_file_location("hmi_make_job", REPO / "tools" / "make_job.py")
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def build_from_config(variant: str | None = None, config_path: str | Path | None = None) -> JobSession:
    """The nominal job of the config (variant) - tools/make_job.py build_nominal: the wall of [[wall.legs]], else a
    straight wall of 24 stones. Not saved."""
    try:
        cfg = mconfig.load(config_path, variant or None)
        job = make_job_tool().build_nominal(cfg, config_path=Path(config_path) if config_path else None)
    except (OSError, ValueError, KeyError, mjob.JobError) as e:
        raise SessionError(f"building the nominal job of config {variant or 'main'!r} failed: {e}") from e
    shape = f"{job.meta.get('shape', 'legs')}" if job.legs else f"L{job.meta.get('length_stones', '')}"
    return JobSession(cfg, mconfig.variant_of(cfg), job, None, "built", f"nominal_{shape}{mconfig.suffix(cfg)}")
