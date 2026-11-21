"""Recording path helpers; no separate export workflow."""
from pathlib import Path


def validate_output_root(output):
    """One selected parent holds independent recording containers with raw data."""
    if not str(output).strip():
        raise ValueError("保存位置不能为空")
    return Path(output).expanduser().resolve()


def resolve_raw_session(source):
    """Accept a new session container, its raw directory, or a legacy session."""
    source = Path(source).expanduser().resolve()
    if not (source / "session.json").is_file() and (source / "raw" / "session.json").is_file():
        return source / "raw"
    return source
