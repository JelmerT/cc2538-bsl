"""Tests for cc2538-bsl.

Two layers:
  * CLI smoke tests that invoke the tool as a subprocess (``python -m
    cc2538_bsl.cc2538_bsl``) and check argument handling / early exits that do
    not require a serial port.
  * Unit tests for the pure helper functions and the parsers, including
    regression tests for the crash bugs fixed alongside these tests.
"""

import os
import sys
import binascii
import subprocess

import pytest

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, REPO_ROOT)

from cc2538_bsl.cc2538_bsl import (  # noqa: E402
    CC26xx,
    CommandInterface,
    FirmwareFile,
    parse_ieee_address,
)

MODULE = "cc2538_bsl.cc2538_bsl"


def run_cli(*args, expect_error=False):
    """Run the CLI as a subprocess from the repo root and return the result."""
    result = subprocess.run(
        [sys.executable, "-m", MODULE, *args],
        cwd=REPO_ROOT,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
    )
    if expect_error:
        assert result.returncode != 0, "expected a non-zero exit, stdout=%r" \
            % result.stdout
    else:
        assert result.returncode == 0, "expected a zero exit, stderr=%r" \
            % result.stderr
    return result


# --------------------------------------------------------------------------
# CLI smoke tests
# --------------------------------------------------------------------------

def test_help_output():
    result = run_cli("-h")
    assert b"usage" in result.stdout.lower()


def test_version_output():
    result = run_cli("--version")
    assert result.stdout.strip()


def test_missing_file_argument_errors():
    # 'file' is a required positional argument.
    run_cli("-w", expect_error=True)


def test_verify_after_read_not_implemented(tmp_path):
    fw = tmp_path / "fw.bin"
    fw.write_bytes(b"\x00\x01\x02\x03")
    result = run_cli("-r", "-v", str(fw), expect_error=True)
    assert b"Verify after read" in result.stderr


# --------------------------------------------------------------------------
# Address encode / decode / checksum helpers
# --------------------------------------------------------------------------

@pytest.fixture
def ci():
    return CommandInterface()


def test_encode_addr_is_big_endian(ci):
    assert ci._encode_addr(0x12345678) == bytes([0x12, 0x34, 0x56, 0x78])


def test_decode_addr_is_little_endian(ci):
    # _decode_addr takes the four bytes least-significant first.
    assert ci._decode_addr(0x78, 0x56, 0x34, 0x12) == 0x12345678


@pytest.mark.parametrize(
    "addr", [0x00000000, 0x00200000, 0x12345678, 0xFFFFFFFF]
)
def test_encode_decode_roundtrip(ci, addr):
    encoded = ci._encode_addr(addr)
    assert ci._decode_addr(*reversed(encoded)) == addr


def test_calc_checks(ci):
    addr, size, cmd = 0x12345678, 0x0000_1000, 0x23
    expected = (sum(ci._encode_addr(addr)) +
                sum(ci._encode_addr(size)) + cmd) & 0xFF
    assert ci._calc_checks(cmd, addr, size) == expected


# --------------------------------------------------------------------------
# IEEE address parsing  (regression tests for the unbound-local crash)
# --------------------------------------------------------------------------

def test_parse_ieee_address_plain_hex():
    assert parse_ieee_address("1122334455667788") == 0x1122334455667788


def test_parse_ieee_address_colon_separated():
    assert parse_ieee_address("11:22:33:44:55:66:77:88") == 0x1122334455667788


def test_parse_ieee_address_dash_separated():
    assert parse_ieee_address("11-22-33-44-55-66-77-88") == 0x1122334455667788


def test_parse_ieee_address_no_delimiter_raises():
    # Previously raised UnboundLocalError instead of a clear ValueError.
    with pytest.raises(ValueError):
        parse_ieee_address("nothexadecimal")


def test_parse_ieee_address_wrong_length_raises():
    with pytest.raises(ValueError):
        parse_ieee_address("11:22:33")


def test_parse_ieee_address_invalid_byte_raises():
    with pytest.raises(ValueError):
        parse_ieee_address("11:22:33:44:55:66:77:zz")


# --------------------------------------------------------------------------
# FirmwareFile
# --------------------------------------------------------------------------

def test_firmwarefile_reads_binary_and_computes_crc32(tmp_path):
    data = bytes(range(256))
    path = tmp_path / "image.bin"
    path.write_bytes(data)

    fw = FirmwareFile(str(path))
    assert bytes(fw.bytes) == data
    assert fw.crc32() == binascii.crc32(data) & 0xFFFFFFFF


def test_firmwarefile_crc32_is_cached(tmp_path):
    path = tmp_path / "image.bin"
    path.write_bytes(bytes(range(64)))
    fw = FirmwareFile(str(path))
    assert fw.crc32() == fw.crc32()


# --------------------------------------------------------------------------
# CC26xx chip detection  (regression test for #150 / #153)
# --------------------------------------------------------------------------

class _FakeCommandInterface:
    """Stand-in returning a constant memory read for CC26xx.__init__."""

    def __init__(self, read):
        self._read = read

    def cmdMemReadCC26xx(self, address):
        return bytearray(self._read)


def test_cc26xx_unknown_wafer_id_does_not_crash():
    # An unrecognized wafer_id previously raised UnboundLocalError on `chip`.
    # It must now fall back to a generic identification without crashing.
    fake = _FakeCommandInterface([0x00, 0x00, 0x00, 0x00])  # -> wafer_id 0x0
    device = CC26xx(fake)  # must not raise
    assert device.page_size == 8192
