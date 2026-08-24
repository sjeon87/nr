# Copyright (c) 2026 University of Peradeniya (UoP)
# Copyright (c) 2026 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)
#
# SPDX-License-Identifier: GPL-2.0-only

# pybind11 struct bindings of the ns3-ai message-interface NR scheduler
# example. The example itself is declared by the parent CMakeLists.txt, so that
# its executable lands in the module examples directory like every other one
# and test.py can find it; only the binding is built here. Both are gated on
# the ai module being available.

# pybind11_add_module and its imported targets are only set up in the directory
# scope that ran find_package(pybind11) (contrib/ai). This example lives under
# contrib/nr, so re-find them here before building the binding.
find_package(Python COMPONENTS Interpreter Development)
find_package(pybind11 CONFIG)

# Neither find_package above is REQUIRED, so a missing dependency leaves
# pybind11_add_module undefined rather than failing. contrib/ai returns early in
# that same case, so the ai module can be listed as enabled while its targets do
# not exist: skip the binding instead of dying with "Unknown CMake command".
if(Python_FOUND AND pybind11_FOUND)
  # pybind11 module mirroring the observation/action structs, dropped next to
  # the Python driver so it can be imported as ns3ai_nr_sched_py.
  pybind11_add_module(ns3ai_nr_sched_py gsoc-nr-ai-sched_py.cc)
  # The binding pulls in ns-3 core symbols (via ai-module.h), so link the ns-3
  # libraries it references (mirrors the ai gym binding linking ${libcore}).
  target_link_libraries(ns3ai_nr_sched_py PRIVATE ${libcore} ${libai})
  set_target_properties(
    ns3ai_nr_sched_py
    PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
  )
  # The binding needs the nr/ai headers generated first, and the Python driver
  # launches the C++ example alongside the binding, so tie them together.
  add_dependencies(ns3ai_nr_sched_py ${libnr} ${libai})
  # The example target is skipped when module examples are filtered out
  # (NS3_FILTER_MODULE_EXAMPLES_AND_TESTS), so only tie it in when it exists.
  if(TARGET gsoc-nr-ai-sched)
    add_dependencies(gsoc-nr-ai-sched ns3ai_nr_sched_py)
  endif()
else()
  message(
    ${HIGHLIGHTED_STATUS}
    "Skipping gsoc-nr-ai-sched bindings: Python or pybind11 not found"
  )
endif()
