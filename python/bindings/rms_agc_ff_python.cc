/*
 * Copyright 2026 Free Software Foundation, Inc.
 *
 * This file is part of GNU Radio
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * F4TNK: single-pass float RMS AGC C++ block binding
 */

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

namespace py = pybind11;

#include <satellites/rms_agc_ff.h>
#include <rms_agc_ff_pydoc.h>

void bind_rms_agc_ff(py::module& m)
{
    using rms_agc_ff = ::gr::satellites::rms_agc_ff;

    py::class_<rms_agc_ff,
               gr::sync_block,
               gr::block,
               gr::basic_block,
               std::shared_ptr<rms_agc_ff>>(m, "rms_agc_ff", D(rms_agc_ff))
        .def(py::init(&rms_agc_ff::make),
             py::arg("alpha") = 0.01f,
             py::arg("reference") = 0.5f,
             D(rms_agc_ff, make))
        .def("set_alpha", &rms_agc_ff::set_alpha, py::arg("alpha"),
             D(rms_agc_ff, set_alpha))
        .def("set_reference", &rms_agc_ff::set_reference, py::arg("reference"),
             D(rms_agc_ff, set_reference));
}
