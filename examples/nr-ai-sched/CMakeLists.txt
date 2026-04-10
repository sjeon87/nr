# Copyright (c) 2026 University of Moratuwa
# Author: Nipuna Dulara
# Copyright (c) 2026 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)
#
# SPDX-License-Identifier: GPL-2.0-only

build_lib_example(
    NAME nr-ai-sched-msg
    SOURCE_FILES
        nr-ai-sched-msg.cc
    LIBRARIES_TO_LINK
        ${libnr}
        ${libai}
        ${libflow-monitor}
)
# Build the pybind11 module only if Python development libs are available.
# On macOS/Linux without python3-dev, this is safely skipped.
find_package(Python COMPONENTS Development.Module QUIET)
if(Python_Development.Module_FOUND)
    pybind11_add_module(ns3ai_nr_sched_py nr_ai_sched_msg_py.cc)
    set_target_properties(ns3ai_nr_sched_py PROPERTIES
        LIBRARY_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/)
    target_link_libraries(ns3ai_nr_sched_py PRIVATE ${libai})
    add_dependencies(nr-ai-sched-msg ns3ai_nr_sched_py)
else()
    message(WARNING
        "Python Development.Module not found — "
        "ns3ai_nr_sched_py pybind11 module will NOT be built. "
        "Install python3-dev to enable it.")
endif()