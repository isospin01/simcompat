"""Capture text that C extensions print directly to the process's stdout/stderr file descriptors.

PyBullet, for example, prints importer warnings from C++, which Python-level redirection cannot
catch. We redirect file descriptors 1 and 2 to a temporary file for the duration of the block.
"""

from __future__ import annotations

import contextlib
import os
import sys
import tempfile
from collections.abc import Iterator


def _flush_c_stdio() -> None:
    with contextlib.suppress(Exception):  # not available on every platform
        import ctypes

        ctypes.CDLL(None).fflush(None)


@contextlib.contextmanager
def captured_native_output() -> Iterator[dict]:
    """``with captured_native_output() as out: ...`` then ``out["text"]`` holds what was printed."""
    out = {"text": ""}
    streams = []
    for stream in (sys.stdout, sys.stderr):
        with contextlib.suppress(Exception):  # e.g. replaced streams under pytest or in notebooks
            streams.append((stream, stream.fileno()))
    if not streams:
        yield out
        return
    with tempfile.TemporaryFile(mode="w+b") as sink:
        saved = []
        for stream, fd in streams:
            stream.flush()
            saved.append((stream, fd, os.dup(fd)))
            os.dup2(sink.fileno(), fd)
        try:
            yield out
        finally:
            _flush_c_stdio()
            for stream, fd, original in saved:
                stream.flush()
                os.dup2(original, fd)
                os.close(original)
            sink.seek(0)
            out["text"] = sink.read().decode("utf-8", "replace")
