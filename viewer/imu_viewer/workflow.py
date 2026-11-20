"""Recording/export orchestration; acquisition and data conversion stay independent."""
from pathlib import Path
from .storage import export_euroc


def validate_output_root(output):
    """One selected parent holds per-recording containers with raw/export siblings."""
    if not str(output).strip():
        raise ValueError("保存位置不能为空")
    return Path(output).expanduser().resolve()


def resolve_raw_session(source):
    """Accept a new session container, its raw directory, or a legacy session."""
    source = Path(source).expanduser().resolve()
    if not (source / "session.json").is_file() and (source / "raw" / "session.json").is_file():
        return source / "raw"
    return source


def next_export_destination(source):
    source = resolve_raw_session(source)
    # Legacy sessions have files directly in their directory: put their export
    # beside them, never inside raw data. New recordings use sibling raw/export.
    base = "export" if source.name == "raw" else source.name + "_euroc"
    destination = source.parent / base
    suffix = 0
    # Never replace a completed or incomplete export. The exporter also uses
    # mkdir(exist_ok=False), so a concurrent creator cannot be overwritten.
    while destination.exists():
        suffix += 1
        destination = source.parent / f"{base}_{suffix:03d}"
    return destination


def export_recording(source):
    """Called after Recorder.done; exporter verifies finalization itself.

    Failed exports keep raw data untouched and retain EXPORT_INCOMPLETE when
    conversion has begun. No cleanup or automatic retry hides a failure.
    """
    source = resolve_raw_session(source)
    destination = next_export_destination(source)
    try:
        result = export_euroc(source, destination)
    except Exception as exc:
        raise RuntimeError(
            f"导出失败；原始会话保留在 {source}；目标 {destination}；"
            f"原因: {type(exc).__name__}: {exc}。请检查后使用‘重新导出’重试。"
        ) from exc
    return dict(result, source=str(source))
