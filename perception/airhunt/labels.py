"""Resolve dataset/detector aliases to the existing AirSim class vocabulary."""

ALIASES = {
    "swimming pool": "pool",
    "trashbin": "trash can",
    "trash bin": "trash can",
    "garbage bin": "trash can",
    "garbage can": "trash can",
}


def canonical_object_label(label, classes):
    normalized = " ".join(label.strip().lower().rstrip(".").split())
    normalized = ALIASES.get(normalized, normalized)
    return next((name for name in classes if name.lower() == normalized), None)


def logger_object_id(value):
    """Map numeric and opaque dataset IDs to stable integers."""
    import zlib

    text = str(value)
    suffix = text.rsplit("_", 1)[-1]
    if "_" in text and suffix.isdecimal():
        return int(suffix)
    return zlib.crc32(text.encode("utf-8"))
