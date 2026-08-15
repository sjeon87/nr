Copyright (c) 2026 University of Peradeniya (UoP)
Copyright (c) 2026 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)

SPDX-License-Identifier: GPL-2.0-only


nr-ai-sched Bindings Installation
=================================

This chapter describes how to build and verify ``ns3ai_nr_sched_py``, the
pybind11 module that mirrors the ns3-ai message-interface scheduler structs
declared in ``contrib/nr/model/nr-mac-scheduler-ai-msg-structs.h``.

Tested with ns-3 3.48, nr v4.2, ns3-ai v1.45.2 (versions pinned by
``ns-3-allinone/MANIFEST.md``) and pybind11 3.0.4 on Python 3.12 and 3.14.
Any interpreter supported by pybind11 >= 2.12 works; the steps below use
Python 3.12, the stock interpreter on Ubuntu 24.04.

Installation Steps for Ubuntu
=============================

Prerequisites
-------------

Update your system and install the required packages:

.. code-block:: bash

   sudo apt update && sudo apt upgrade -y
   sudo apt install -y \
       build-essential git cmake ninja-build pkg-config \
       python3.12 python3.12-venv python3.12-dev \
       libboost-program-options-dev \
       libprotobuf-dev protobuf-compiler \
       libsqlite3-dev libxml2-dev libgsl-dev

The ``ai`` module requires Boost.program_options, Python development
headers, Protobuf, and pybind11 (installed in the next step). If any of the
four is missing, ``contrib/ai`` is silently dropped from the build and these
bindings are skipped.

Set Up the Python Environment
-----------------------------

From the ns-3 root directory, create and activate a virtual environment,
then install pybind11, the only Python-side requirement:

.. code-block:: bash

   python3.12 -m venv .venv
   source .venv/bin/activate
   python -m pip install --upgrade pip
   python -m pip install -r contrib/nr/examples/nr-ai-sched/requirements.txt

.. note::

   The virtual environment must remain active for all subsequent ``./ns3``
   commands. CMake binds the interpreter at the *first* configure of a given
   cache; activating a different environment later has no effect until
   ``./ns3 clean`` is run.

Configure and Build
-------------------

Configure from within the active environment, then build. The
``-Dpybind11_DIR`` argument is required, not a fallback:
``find_package(pybind11 CONFIG)`` does not search a virtual environment's
site-packages, so without the flag configure reports ``Skipping contrib/ai:
pybind11 not found`` and the bindings are never built. (A second configure
then appears to succeed because an unrelated pybind11 lookup populates the
cache in the meantime; do not rely on it.)

.. code-block:: bash

   ./ns3 configure --build-profile debug --enable-examples --enable-tests -- \
       -Dpybind11_DIR=$(python -m pybind11 --cmakedir)
   ./ns3 build nr nr-ai-sched

The module list printed by configure must contain ``ai`` and ``nr``. If
CMake still selects a different interpreter than the one you activated, run
``./ns3 clean`` and configure again, adding:

.. code-block:: bash

       -DPython_EXECUTABLE=$PWD/.venv/bin/python3 \
       -DPython3_EXECUTABLE=$PWD/.venv/bin/python3

The built module is written into ``contrib/nr/examples/nr-ai-sched/``, named
after the ABI it targets, for example
``ns3ai_nr_sched_py.cpython-312-x86_64-linux-gnu.so``.

Run the Tests
-------------

Once the build completes, run the binding smoke tests with the same
interpreter:

.. code-block:: bash

   python contrib/nr/examples/nr-ai-sched/test_bindings.py

Expected Output
===============

.. code-block:: text

   .......
   ----------------------------------------------------------------------
   Ran 7 tests in 0.001s

   OK

The suite checks that every struct and field declared in the header is
exposed, that each field round-trips a value, that ``get_lc()``
bounds-checks its index and returns a reference into the parent observation,
and that the ``Ns3AiMsgInterfaceImpl`` accessor API is present. The tests do
not touch shared memory; the live handshake is exercised by running the
nr-ai-sched example end to end.

Failure output
--------------

If the module was not built, or you run a different interpreter from the one
it was built against, the tests fail at import:

.. code-block:: text

   ModuleNotFoundError: No module named 'ns3ai_nr_sched_py'

Check whether the module exists and matches your interpreter:

.. code-block:: bash

   ls contrib/nr/examples/nr-ai-sched/ns3ai_nr_sched_py*.so
   python -c "import sys; print(sys.version_info[:2])"

If the module does not exist, ``ai`` was not enabled at configure time; see
Configure and Build above. If its ABI tag disagrees with your interpreter (a
``cpython-39`` module cannot be imported by Python 3.12), the cache predates
your environment: check ``PYTHON_MODULE_EXTENSION`` in
``cmake-cache/CMakeCache.txt``, then run ``./ns3 clean`` and configure
again; reconfiguring alone does not change it.

Stale modules are worse than an import error: ``.so`` files accumulate in
the source directory, survive branch switches, and are not removed by
``./ns3 clean``. One that predates a struct-layout change imports
successfully and silently misreads the shared memory. After switching
branches or changing the struct layout:

.. code-block:: bash

   rm -f contrib/nr/examples/nr-ai-sched/ns3ai_nr_sched_py*.so

When running the full example, the shared-memory segment names must match:
the simulation attaches to ``"ns3-ai_" + trialName`` (default
``ns3-ai_single_trial``), while the ns3-ai Python helper creates ``ns3-ai``
by default. On a mismatch, or when the simulation is launched without the
Python driver (which must create the segment before the C++ side can
attach), the simulation aborts with
``boost::interprocess_exception: No such file or directory`` and the Python
side hangs.
