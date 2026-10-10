"""Errors caused by the user's input or options.

Every such error derives from :class:`SimCompatError`. The command line prints its message (which
always says what to do) and exits with code 2, without a traceback. Any other exception is a bug
in simcompat and keeps its traceback.
"""

from __future__ import annotations


class SimCompatError(ValueError):
    """Base class for problems with the user's input or options."""


class UsageError(SimCompatError):
    """Invalid options: an unknown check pattern, reference, output format, ..."""
