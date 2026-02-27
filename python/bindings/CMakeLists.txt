# Copyright 2020 Free Software Foundation, Inc.
#
# This file is part of GNU Radio
#
# SPDX-License-Identifier: GPL-3.0-or-later
#

########################################################################
# Check if there is C++ code at all
########################################################################
if(NOT satellites_sources)
    MESSAGE(STATUS "No C++ sources... skipping python bindings")
    return()
endif(NOT satellites_sources)

########################################################################
# Check for pygccxml
########################################################################
GR_PYTHON_CHECK_MODULE_RAW(
    "pygccxml"
    "import pygccxml"
    PYGCCXML_FOUND
    )

include(GrPybind)

########################################################################
# Python Bindings
########################################################################

list(APPEND satellites_python_files
    ax100_decode_python.cc
    crc_python.cc
    crc_append_python.cc
    crc_check_python.cc
    convolutional_encoder_python.cc
    costas_loop_8apsk_cc_python.cc
    decode_ra_code_python.cc
    decode_rs_python.cc
    descrambler308_python.cc
    distributed_syncframe_soft_python.cc
    doppler_correction_python.cc
    encode_rs_python.cc
    fixedlen_to_pdu_python.cc
    frame_counter_python.cc
    hdlc_deframer_python.cc
    kiss_to_pdu_python.cc
    kurtosis_python.cc
    level_to_message_python.cc
    lilacsat1_demux_python.cc
    manchester_sync_python.cc
    matrix_deinterleaver_soft_python.cc
    message_counter_python.cc
    nanocom_golay_decode_length_python.cc
    nrzi_decode_python.cc
    nrzi_encode_python.cc
    nusat_decoder_python.cc
    rms_agc_cc_python.cc
    rms_agc_ff_python.cc
    packet_csma_python.cc
    pdu_add_meta_python.cc
    pdu_head_tail_python.cc
    pdu_length_filter_python.cc
    pdu_scrambler_python.cc
    phase_unwrap_python.cc
    python_bindings.cc
    selector_python.cc
    time_dependent_delay_python.cc
    u482c_decode_python.cc
    u482c_encode_python.cc
    varlen_packet_framer_python.cc
    varlen_packet_tagger_python.cc
    viterbi_decoder_python.cc
)

GR_PYBIND_MAKE_OOT(satellites 
   ../..
   gr::satellites
   "${satellites_python_files}")

# F4TNK: copy pydoc templates from docstrings/ to the binary dir so the new
# bindings can find their *_pydoc.h headers at compile time.
foreach(_f hdlc_deframer kiss_to_pdu rms_agc_cc rms_agc_ff)
    if(NOT EXISTS "${CMAKE_CURRENT_BINARY_DIR}/${_f}_pydoc.h")
        configure_file(
            "${CMAKE_CURRENT_SOURCE_DIR}/docstrings/${_f}_pydoc_template.h"
            "${CMAKE_CURRENT_BINARY_DIR}/${_f}_pydoc.h"
            COPYONLY)
    endif()
endforeach()

install(TARGETS satellites_python DESTINATION ${GR_PYTHON_DIR}/satellites COMPONENT pythonapi)
