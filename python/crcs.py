#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# Copyright 2022 Daniel Estevez <daniel@destevez.net>
#
# This file is part of gr-satellites
#
# SPDX-License-Identifier: GPL-3.0-or-later
#

from gnuradio import gr

# F4TNK: Always use gr-satellites crc_check which includes
# 1-bit-flip error correction retry on CRC failure.
# The gr-satellites version is API-compatible with gnuradio.digital.crc_check
# but adds automatic single-bit error recovery.
from . import crc_check


def crc16_arc(swap_endianness=True, discard_crc=True):
    return crc_check(16, 0x8005, 0x0, 0x0, True, True,
                     swap_endianness, discard_crc)


def crc16_ccitt_x25(swap_endianness=True, discard_crc=True):
    return crc_check(16, 0x1021, 0xFFFF, 0xFFFF, True, True,
                     swap_endianness, discard_crc)


def crc16_ccitt_false(swap_endianness=False, discard_crc=True):
    return crc_check(16, 0x1021, 0xFFFF, 0x0, False, False,
                     swap_endianness, discard_crc)


def crc16_ccitt_zero(swap_endianness=False, discard_crc=True):
    return crc_check(16, 0x1021, 0x0, 0x0, False, False,
                     swap_endianness, discard_crc)


def crc16_cc11xx(discard_crc=True):
    return crc_check(16, 0x8005, 0xFFFF, 0x0, False, False,
                     False, discard_crc)


def crc32c(discard_crc=True):
    return crc_check(32, 0x1EDC6F41, 0xFFFFFFFF, 0xFFFFFFFF,
                     True, True, False, discard_crc)
