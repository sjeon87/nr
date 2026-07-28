# Copyright (c) 2026 University of Peradeniya (UoP)
# Copyright (c) 2026 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)
#
# SPDX-License-Identifier: GPL-2.0-only

# ns3-ai message-interface NR scheduler example: the gsoc-nr-rl-based-sched
# scenario driven over shared memory, plus the pybind11 struct bindings.
# Built only when the ai module is available (the parent gates this with
# add_subdirectory under the same condition).

build_lib_example(
  NAME nr-ai-sched
  SOURCE_FILES nr-ai-sched.cc
  LIBRARIES_TO_LINK ${libnr}
                    ${libflow-monitor}
                    ${libai}
)

# pybind11_add_module and its imported targets are only set up in the directory
# scope that ran find_package(pybind11) (contrib/ai). This example lives under
# contrib/nr, so re-find them here before building the binding.
find_package(Python COMPONENTS Interpreter Development)
find_package(pybind11 CONFIG)

# pybind11 module mirroring the observation/action structs, dropped next to the
# Python driver so it can be imported as ns3ai_nr_sched_py.
pybind11_add_module(ns3ai_nr_sched_py nr-ai-sched_py.cc)
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
add_dependencies(nr-ai-sched ns3ai_nr_sched_py)
