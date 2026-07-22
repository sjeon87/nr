// Copyright (c) 2026 University of Peradeniya (UoP)
// Copyright (c) 2026 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)
//
// SPDX-License-Identifier: GPL-2.0-only

// pybind11 bindings for the NR scheduler ns3-ai message-interface structs and
// the vector-mode message interface. Mirrors the layout of
// contrib/nr/model/nr-mac-scheduler-ai-msg-structs.h so a Python agent can read
// observations and write actions in shared memory with zero serialization.

#include "ns3/ai-module.h"
#include "ns3/nr-mac-scheduler-ai-msg-structs.h"

#include <iostream>
#include <pybind11/pybind11.h>

namespace py = pybind11;
using namespace ns3;

using Impl = Ns3AiMsgInterfaceImpl<NrSchedulerObservation, NrSchedulerAction>;
using ObsVector = Impl::Cpp2PyMsgVector;
using ActVector = Impl::Py2CppMsgVector;

PYBIND11_MAKE_OPAQUE(ObsVector);
PYBIND11_MAKE_OPAQUE(ActVector);

PYBIND11_MODULE(ns3ai_nr_sched_py, m)
{
    py::class_<NrSchedulerLcObservation>(m, "NrSchedulerLcObservation")
        .def(py::init<>())
        .def_readwrite("holDelay", &NrSchedulerLcObservation::holDelay)
        .def_readwrite("delayBudgetMs", &NrSchedulerLcObservation::delayBudgetMs)
        .def_readwrite("lcId", &NrSchedulerLcObservation::lcId)
        .def_readwrite("fiveQI", &NrSchedulerLcObservation::fiveQI)
        .def_readwrite("priority", &NrSchedulerLcObservation::priority)
        .def_readwrite("resourceType", &NrSchedulerLcObservation::resourceType)
        .def_readwrite("bsr", &NrSchedulerLcObservation::bsr);

    py::class_<NrSchedulerObservation>(m, "NrSchedulerObservation")
        .def(py::init<>())
        .def_readwrite("cqi", &NrSchedulerObservation::cqi)
        .def_readwrite("avgTput", &NrSchedulerObservation::avgTput)
        .def_readwrite("potentialTput", &NrSchedulerObservation::potentialTput)
        .def_readwrite("assignedBytes", &NrSchedulerObservation::assignedBytes)
        .def_readwrite("rnti", &NrSchedulerObservation::rnti)
        .def_readwrite("numLcs", &NrSchedulerObservation::numLcs)
        .def(
            "get_lc",
            [](NrSchedulerObservation& o, std::size_t i) -> NrSchedulerLcObservation& {
                if (i >= MAX_LCS_PER_UE)
                {
                    throw py::index_error();
                }
                return o.lc[i];
            },
            py::return_value_policy::reference_internal);

    py::class_<NrSchedulerAction>(m, "NrSchedulerAction")
        .def(py::init<>())
        .def_readwrite("rnti", &NrSchedulerAction::rnti)
        .def_readwrite("weight", &NrSchedulerAction::weight);

    py::class_<ObsVector>(m, "NrSchedulerObsVector")
        .def("resize", static_cast<void (ObsVector::*)(ObsVector::size_type)>(&ObsVector::resize))
        .def("__len__", &ObsVector::size)
        .def(
            "__getitem__",
            [](ObsVector& vec, uint32_t i) -> NrSchedulerObservation& {
                if (i >= vec.size())
                {
                    std::cerr << "Invalid index " << i << " for obs vector of size " << vec.size()
                              << std::endl;
                    exit(1);
                }
                return vec.at(i);
            },
            py::return_value_policy::reference);

    py::class_<ActVector>(m, "NrSchedulerActVector")
        .def("resize", static_cast<void (ActVector::*)(ActVector::size_type)>(&ActVector::resize))
        .def("__len__", &ActVector::size)
        .def(
            "__getitem__",
            [](ActVector& vec, uint32_t i) -> NrSchedulerAction& {
                if (i >= vec.size())
                {
                    std::cerr << "Invalid index " << i << " for act vector of size " << vec.size()
                              << std::endl;
                    exit(1);
                }
                return vec.at(i);
            },
            py::return_value_policy::reference);

    py::class_<Impl>(m, "Ns3AiMsgInterfaceImpl")
        .def(py::init<bool,
                      bool,
                      bool,
                      uint32_t,
                      const char*,
                      const char*,
                      const char*,
                      const char*>())
        .def("PyRecvBegin", &Impl::PyRecvBegin)
        .def("PyRecvEnd", &Impl::PyRecvEnd)
        .def("PySendBegin", &Impl::PySendBegin)
        .def("PySendEnd", &Impl::PySendEnd)
        .def("PyGetFinished", &Impl::PyGetFinished)
        .def("GetCpp2PyVector", &Impl::GetCpp2PyVector, py::return_value_policy::reference)
        .def("GetPy2CppVector", &Impl::GetPy2CppVector, py::return_value_policy::reference);
}
