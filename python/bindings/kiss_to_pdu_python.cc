/*
 * Copyright 2024 F4TNK
 *
 * This file is part of gr-satellites
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 */

#include <pybind11/complex.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

namespace py = pybind11;

#include <satellites/kiss_to_pdu.h>
// For pydoc strings
#include "kiss_to_pdu_pydoc.h"

void bind_kiss_to_pdu(py::module& m)
{
    using kiss_to_pdu = ::gr::satellites::kiss_to_pdu;

    py::class_<kiss_to_pdu,
               gr::sync_block,
               gr::block,
               gr::basic_block,
               std::shared_ptr<kiss_to_pdu>>(m, "kiss_to_pdu", D(kiss_to_pdu))

        .def(py::init(&kiss_to_pdu::make),
             py::arg("control_byte") = true,
             D(kiss_to_pdu, make));
}
