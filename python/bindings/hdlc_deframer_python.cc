/*
 * Copyright 2025 F4TNK
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

#include <satellites/hdlc_deframer.h>
// pydoc.h is automatically generated in the build directory
#include <hdlc_deframer_pydoc.h>

void bind_hdlc_deframer(py::module& m)
{

    using hdlc_deframer = ::gr::satellites::hdlc_deframer;


    py::class_<hdlc_deframer,
               gr::sync_block,
               gr::block,
               gr::basic_block,
               std::shared_ptr<hdlc_deframer>>(m, "hdlc_deframer", D(hdlc_deframer))

        .def(py::init(&hdlc_deframer::make),
             py::arg("check_fcs"),
             py::arg("max_length"),
             D(hdlc_deframer, make))


        ;
}
