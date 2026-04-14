// Copyright (c) 2026 University of Moratuwa
// Author: Nipuna Dulara
// Copyright (c) 2026 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)
//
// SPDX-License-Identifier: GPL-2.0-only
/**
 * @file nr_ai_sched_msg_py.cc
 * @brief Pybind11 module exposing NR scheduler shared memory structs to Python.
 *
 * This file builds a Python extension module called ns3ai_nr_sched_py.
 * The Python PPO agent imports this module to read observations from and
 * write actions to the shared memory segment created by the C++ simulation.
 */
#include "ns3/ai-module.h"
// Include the shared memory struct definitions from the NR model
#include "../../model/nr-mac-scheduler-ai-msg-structs.h"

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
namespace py = pybind11;

PYBIND11_MODULE(ns3ai_nr_sched_py, m)
{
    m.doc() = "Pybind11 bindings for NR AI scheduler shared memory interface";
    // Observation struct (C++ to Python, read only for Python)
    py::class_<ns3::NrSchedObservation>(m, "NrSchedObservation")
        .def(py::init<>())
        .def_readwrite("rnti", &ns3::NrSchedObservation::rnti)
        .def_readwrite("lcId", &ns3::NrSchedObservation::lcId)
        .def_readwrite("fiveQi", &ns3::NrSchedObservation::fiveQi)
        .def_readwrite("priority", &ns3::NrSchedObservation::priority)
        .def_readwrite("holDelay", &ns3::NrSchedObservation::holDelay)
        .def_readwrite("cqi", &ns3::NrSchedObservation::cqi)
        .def_readwrite("bsr", &ns3::NrSchedObservation::bsr)
        .def_readwrite("avgTput", &ns3::NrSchedObservation::avgTput)
        .def_readwrite("potentialTput", &ns3::NrSchedObservation::potentialTput);
    // Action struct (Python to C++, written by Python)
    py::class_<ns3::NrSchedAction>(m, "NrSchedAction")
        .def(py::init<>())
        .def_readwrite("rnti", &ns3::NrSchedAction::rnti)
        .def_readwrite("lcId", &ns3::NrSchedAction::lcId)
        .def_readwrite("weight", &ns3::NrSchedAction::weight);
    // Envelope: C++ to Python
    py::class_<ns3::NrSchedEnvMsg>(m, "NrSchedEnvMsg")
        .def(py::init<>())
        .def_readwrite("numFlows", &ns3::NrSchedEnvMsg::numFlows)
        .def_readwrite("reward", &ns3::NrSchedEnvMsg::reward)
        .def_readwrite("isFinished", &ns3::NrSchedEnvMsg::isFinished)
        .def(
            "get_obs",
            [](ns3::NrSchedEnvMsg& self, uint32_t idx) -> ns3::NrSchedObservation& {
                if (idx >= ns3::MAX_FLOWS)
                {
                    throw py::index_error("Index out of range");
                }
                return self.obs[idx];
            },
            py::return_value_policy::reference_internal,
            py::arg("idx"),
            "Get observation at index idx (0-based)");

    // Envelope: Python to C++ (actions)
    py::class_<ns3::NrSchedActMsg>(m, "NrSchedActMsg")
        .def(py::init<>())
        .def_readwrite("numFlows", &ns3::NrSchedActMsg::numFlows)
        .def(
            "get_action",
            [](ns3::NrSchedActMsg& self, uint32_t idx) -> ns3::NrSchedAction& {
                if (idx >= ns3::MAX_FLOWS)
                {
                    throw py::index_error("Index out of range");
                }
                return self.actions[idx];
            },
            py::return_value_policy::reference_internal,
            py::arg("idx"),
            "Get action slot at index idx (0-based)");
    // Message interface implementation (semaphore sync for Python side)
    using MsgImpl = ns3::Ns3AiMsgInterfaceImpl<ns3::NrSchedEnvMsg, ns3::NrSchedActMsg>;
    py::class_<MsgImpl>(m, "Ns3AiMsgInterfaceImpl")
        .def(py::init<bool,
                      bool,
                      bool,
                      uint32_t,
                      const char*,
                      const char*,
                      const char*,
                      const char*>(),
             py::arg("is_memory_creator"),
             py::arg("use_vector"),
             py::arg("handle_finish"),
             py::arg("size"),
             py::arg("segment_name"),
             py::arg("cpp2py_msg_name"),
             py::arg("py2cpp_msg_name"),
             py::arg("lockable_name"))
        .def("PyRecvBegin", &MsgImpl::PyRecvBegin, "Block until C++ sends observations")
        .def("PyRecvEnd", &MsgImpl::PyRecvEnd, "Signal C++ that Python finished reading")
        .def("PySendBegin", &MsgImpl::PySendBegin, "Acquire lock to write actions")
        .def("PySendEnd", &MsgImpl::PySendEnd, "Signal C++ that actions are ready")
        .def("PyGetFinished", &MsgImpl::PyGetFinished, "Check if the simulation has ended")
        .def("GetCpp2PyStruct",
             &MsgImpl::GetCpp2PyStruct,
             py::return_value_policy::reference,
             "Get pointer to the observation envelope")
        .def("GetPy2CppStruct",
             &MsgImpl::GetPy2CppStruct,
             py::return_value_policy::reference,
             "Get pointer to the action envelope");
}
