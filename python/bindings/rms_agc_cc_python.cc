/*
 * Copyright 2026 Free Software Foundation, Inc.
 *
 * This file is part of GNU Radio
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * F4TNK: single-pass RMS AGC C++ block binding
 */

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

namespace py = pybind11;

#include <satellites/rms_agc_cc.h>
#include <rms_agc_cc_pydoc.h>

void bind_rms_agc_cc(py::module& m)
{
    using rms_agc_cc = ::gr::satellites::rms_agc_cc;

    py::class_<rms_agc_cc,
               gr::sync_block,
               gr::block,
               gr::basic_block,
               std::shared_ptr<rms_agc_cc>>(m, "rms_agc_cc", D(rms_agc_cc))
        .def(py::init(&rms_agc_cc::make),
             py::arg("alpha") = 0.01f,
             py::arg("reference") = 0.5f,
             D(rms_agc_cc, make))
        .def("set_alpha", &rms_agc_cc::set_alpha, py::arg("alpha"),
             D(rms_agc_cc, set_alpha))
        .def("set_reference", &rms_agc_cc::set_reference, py::arg("reference"),
             D(rms_agc_cc, set_reference));
}
