# Copyright (c) 2026 University of Peradeniya (UoP)
# Copyright (c) 2026 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)
#
# SPDX-License-Identifier: GPL-2.0-only

"""Smoke tests for the ns3ai_nr_sched_py pybind11 bindings.

Guards the binary contract documented in
contrib/nr/model/nr-mac-scheduler-ai-msg-structs.h where every struct field must be
mirrored by the bindings. The suite checks that the module imports, that every
field declared in the header is exposed and round-trips a value, and that
get_lc() bounds-checks and returns a reference into the parent observation.

The Ns3AiMsgInterfaceImpl handshake itself is not exercised here: Python is
the shared-memory accessor, so it needs a live C++ creator (covered by
running the gsoc-nr-ai-sched example end to end).

Run manually with the same Python interpreter the bindings were compiled
against (the one active when CMake configured the build. A mismatch shows
up as an ImportError for the .so):

    python contrib/nr/examples/gsoc-nr-ai-sched/test_bindings.py
"""

import os
import re
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import ns3ai_nr_sched_py as bindings

HEADER = os.path.join(
    os.path.dirname(os.path.abspath(__file__)),
    "..",
    "..",
    "model",
    "nr-mac-scheduler-ai-msg-structs.h",
)

# The per-bearer array is exposed through get_lc() rather than as an attribute.
FIELD_TO_ACCESSOR = {("NrSchedulerObservation", "lc"): "get_lc"}


def parse_header_fields(path):
    """Extract {struct name: [field names]} from the msg-structs header."""
    with open(path, encoding="utf-8") as f:
        text = f.read()
    text = re.sub(r"//.*", "", text)
    text = re.sub(r"/\*.*?\*/", "", text, flags=re.DOTALL)
    structs = {}
    for name, body in re.findall(r"struct\s+(\w+)\s*\{(.*?)\};", text, flags=re.DOTALL):
        structs[name] = re.findall(r"[\w:]+\s+(\w+)\s*(?:\[\w+\])?\s*;", body)
    return structs


def max_lcs_per_ue(path):
    with open(path, encoding="utf-8") as f:
        match = re.search(r"MAX_LCS_PER_UE\s*=\s*(\d+)", f.read())
    return int(match.group(1))


class BindingCompleteness(unittest.TestCase):
    """Every struct and field in the header must be exposed by the module."""

    def test_all_header_fields_are_bound(self):
        structs = parse_header_fields(HEADER)
        self.assertGreaterEqual(len(structs), 3, "header parse found too few structs")
        for struct_name, fields in structs.items():
            cls = getattr(bindings, struct_name, None)
            self.assertIsNotNone(cls, f"{struct_name} is not exposed by the bindings")
            obj = cls()
            for field in fields:
                accessor = FIELD_TO_ACCESSOR.get((struct_name, field), field)
                self.assertTrue(
                    hasattr(obj, accessor),
                    f"{struct_name}.{field} exists in the header but is not bound"
                    " (new field added without updating gsoc-nr-ai-sched_py.cc?)",
                )


class FieldRoundTrip(unittest.TestCase):
    """Set and read back every bound field with a type-appropriate value."""

    # Float fields use values exactly representable in float32.
    LC_VALUES = {
        "holDelay": 12,
        "delayBudgetMs": 100,
        "lcId": 4,
        "fiveQI": 9,
        "priority": 70,
        "resourceType": 1,
        "bsr": 1536.5,
    }
    OBS_VALUES = {
        "cqi": 11.0,
        "avgTput": 2.5,
        "potentialTput": 8.25,
        "assignedBytes": 4096,
        "rnti": 42,
        "numLcs": 2,
    }
    ACT_VALUES = {
        "rnti": 42,
        "weight": 0.75,
    }

    def round_trip(self, obj, values):
        for field, value in values.items():
            setattr(obj, field, value)
        for field, value in values.items():
            self.assertEqual(getattr(obj, field), value, f"{field} did not round-trip")

    def test_lc_observation(self):
        self.round_trip(bindings.NrSchedulerLcObservation(), self.LC_VALUES)

    def test_observation(self):
        self.round_trip(bindings.NrSchedulerObservation(), self.OBS_VALUES)

    def test_action(self):
        self.round_trip(bindings.NrSchedulerAction(), self.ACT_VALUES)


class GetLcSemantics(unittest.TestCase):
    """get_lc() must bounds-check and alias the parent's storage."""

    def test_out_of_range_raises(self):
        obs = bindings.NrSchedulerObservation()
        limit = max_lcs_per_ue(HEADER)
        for i in range(limit):
            obs.get_lc(i)  # valid indices must not raise
        with self.assertRaises(IndexError):
            obs.get_lc(limit)

    def test_returns_reference_not_copy(self):
        obs = bindings.NrSchedulerObservation()
        obs.get_lc(0).lcId = 7
        self.assertEqual(obs.get_lc(0).lcId, 7)


class MsgInterfaceSurface(unittest.TestCase):
    """The handshake class must expose the accessor-side API (not instantiated
    here: the constructor attaches to shared memory a C++ creator must own)."""

    def test_expected_methods_exist(self):
        for method in (
            "PyRecvBegin",
            "PyRecvEnd",
            "PySendBegin",
            "PySendEnd",
            "PyGetFinished",
            "GetCpp2PyVector",
            "GetPy2CppVector",
        ):
            self.assertTrue(hasattr(bindings.Ns3AiMsgInterfaceImpl, method))


if __name__ == "__main__":
    unittest.main()
