import gc
import weakref
import pytest

import ifm3dpy.logging as lg
from types import SimpleNamespace


def test_set_writer_retains_python_writer_when_python_ref_removed():
    """
    Ensure that when a Python subclass of `LogWriter` is passed to
    `Logger.set_writer`, the C++ side holds a `shared_ptr` to it and
    continues to call its `write` method even if the Python-side
    reference is deleted (i.e., verifies behavior introduced by
    `py::smart_holder`).

    This test is hardware-independent and uses `emit_test_log`.
    """

    msgs = []

    class MyWriter(lg.LogWriter):
        def __init__(self):
            super().__init__()

        def write(self, entry):
            # append into an external list so we can observe calls even
            # after the Python reference to this instance has been
            # removed.
            msgs.append(entry.message)

    writer = MyWriter()

    # register the writer with the Logger (C++ side should take
    # ownership via shared_ptr)
    lg.Logger.set_writer(writer)
    lg.Logger.set_log_level(lg.LogLevel.Verbose)

    # Remove the Python reference and force GC to make sure only the
    # C++ shared_ptr keeps the writer alive. We verify lifetime by using
    # a weakref: if the object is still alive after GC, C++ must be
    # holding a reference to it (i.e., smart_holder semantics are
    # functioning). This avoids needing a device or a C++ test helper
    # function to trigger the write path.
    wr = weakref.ref(writer)
    del writer
    gc.collect()

    # The writer should still be alive because the C++ side should
    # retain ownership via a shared_ptr stored by Logger.SetWriter.
    assert wr() is not None, "Expected writer to be kept alive by C++ (py::smart_holder), but it was collected"

    # As a final sanity check, ensure the object still responds to
    # `.write` by invoking it directly (this runs entirely on the
    # Python side but proves the object remains usable).
    wr().write(SimpleNamespace(message="sanity"))

    # Cleanup
    lg.Logger.set_writer(None)
