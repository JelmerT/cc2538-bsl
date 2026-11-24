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

import cc2538_bsl.cc2538_bsl as bsl  # noqa: E402
from cc2538_bsl.cc2538_bsl import (  # noqa: E402
    CC2538,
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


def test_version_ignores_git_repo_in_cwd(tmp_path):
    git = ["git", "-c", "user.name=test", "-c", "user.email=test@example.invalid"]
    subprocess.run(git + ["init", "-q"], cwd=str(tmp_path), check=True)
    subprocess.run(git + ["commit", "-q", "--allow-empty", "-m", "x"],
                   cwd=str(tmp_path), check=True)
    subprocess.run(git + ["tag", "9.9.9"], cwd=str(tmp_path), check=True)

    result = subprocess.run(
        [sys.executable, "-m", MODULE, "--version"],
        cwd=str(tmp_path),
        env=dict(os.environ, PYTHONPATH=REPO_ROOT),
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
    )
    assert result.returncode == 0
    assert b"9.9.9" not in result.stdout


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


# --------------------------------------------------------------------------
# CLI operation safety  (#139, #187, #191)
# --------------------------------------------------------------------------

class _FakeCmd:
    def __init__(self):
        self.calls = []

    def open(self, *args):
        self.calls.append("open")

    def invoke_bootloader(self, *args):
        pass

    def sendSynch(self):
        return 1

    def cmdGetChipId(self):
        return 0xFFFF

    def writeMemory(self, addr, data, force=False):
        self.calls.append("write")
        self.write_addr = addr
        return 1

    def cmdReset(self):
        self.calls.append("reset")
        return 1


class _FakeDevice(bsl.Chip):
    erase_result = 1

    def __init__(self, command_interface):
        super().__init__(command_interface)
        self.size = 0x1000
        self.has_cmd_set_xosc = False
        self.page_size = 0x100

    def erase(self):
        self.command_interface.calls.append("erase")
        return 1

    def eraseRange(self, addr, size):
        self.command_interface.calls.append("erase_range")
        self.command_interface.erase_args = (addr, size)
        return self.erase_result

    def read_memory(self, addr):
        self.command_interface.calls.append("read")
        self.command_interface.read_addr = addr
        return bytearray(b"\xAA\xBB\xCC\xDD")


@pytest.fixture
def fake_cli(monkeypatch):
    cmd = _FakeCmd()
    monkeypatch.setattr(bsl, "CommandInterface", lambda: cmd)
    monkeypatch.setattr(bsl, "CC26xx", _FakeDevice)
    monkeypatch.setattr(_FakeDevice, "erase_result", 1)

    def run(*args):
        monkeypatch.setattr(sys, "argv", ["cc2538-bsl", "-p", "fake", *args])
        bsl.main_cli()

    run.cmd = cmd
    return run


def test_no_file_allowed_for_non_file_operations(fake_cli):
    fake_cli("-e")
    assert fake_cli.cmd.calls == ["open", "erase", "reset"]


@pytest.mark.parametrize("flag", ["-w", "-W", "-v", "-r"])
def test_file_required_for_file_operations(fake_cli, flag):
    with pytest.raises(SystemExit) as exc:
        fake_cli(flag)
    assert exc.value.code == 2
    assert fake_cli.cmd.calls == []


def test_bad_firmware_does_not_touch_device(fake_cli, tmp_path):
    with pytest.raises(SystemExit):
        fake_cli("-e", "-w", str(tmp_path / "missing.bin"))
    assert fake_cli.cmd.calls == []


def test_empty_firmware_does_not_touch_device(fake_cli, tmp_path):
    fw = tmp_path / "empty.bin"
    fw.write_bytes(b"")
    with pytest.raises(SystemExit) as exc:
        fake_cli("-e", "-w", str(fw))
    assert "empty" in str(exc.value.code)
    assert fake_cli.cmd.calls == []


def test_oversized_firmware_rejected_before_erase(fake_cli, tmp_path):
    fw = tmp_path / "big.bin"
    fw.write_bytes(b"\x01" * 0x1004)
    with pytest.raises(SystemExit) as exc:
        fake_cli("-e", "-w", str(fw))
    assert "past the end of flash" in str(exc.value.code)
    assert "erase" not in fake_cli.cmd.calls


def test_oversized_firmware_allowed_with_force(fake_cli, tmp_path):
    fw = tmp_path / "big.bin"
    fw.write_bytes(b"\x01" * 0x1004)
    fake_cli("-f", "-e", "-w", str(fw))
    assert fake_cli.cmd.calls == ["open", "erase", "write", "reset"]


def test_exact_fit_firmware_allowed(fake_cli, tmp_path):
    fw = tmp_path / "fit.bin"
    fw.write_bytes(b"\x01" * 0x1000)
    fake_cli("-e", "-w", str(fw))
    assert "write" in fake_cli.cmd.calls


def test_erase_write_stops_when_erase_fails(fake_cli, monkeypatch, tmp_path):
    monkeypatch.setattr(_FakeDevice, "erase_result", 0)
    fw = tmp_path / "fw.bin"
    fw.write_bytes(b"\x01" * 16)
    with pytest.raises(SystemExit):
        fake_cli("-W", str(fw))
    assert "write" not in fake_cli.cmd.calls


def test_default_read_stops_at_end_of_flash(fake_cli, tmp_path):
    out = tmp_path / "out.bin"
    fake_cli("-r", str(out))
    assert out.stat().st_size == 0x1000


def test_explicit_read_past_flash_rejected_before_output(fake_cli, tmp_path):
    out = tmp_path / "out.bin"
    out.write_bytes(b"keep")
    with pytest.raises(SystemExit) as exc:
        fake_cli("-r", "-l", str(0x1001), str(out))
    assert "past the end of flash" in str(exc.value.code)
    assert out.read_bytes() == b"keep"


def test_erase_read_write_prompts_only_for_overwrite(fake_cli, monkeypatch, tmp_path):
    fw = tmp_path / "fw.bin"
    fw.write_bytes(b"\x01" * 16)
    questions = []
    monkeypatch.setattr(bsl, "query_yes_no",
                        lambda question, default: questions.append(question) or True)
    fake_cli("-e", "-w", "-r", "-l", "16", str(fw))
    assert len(questions) == 1
    assert "overwrite your input file" in questions[0]


@pytest.mark.parametrize("wafer_bytes", [
    [0x00, 0xE0, 0x9B, 0x5B],  # CC13xx family, unlisted PG nibble
    [0x00, 0xA0, 0x99, 0x5B],  # CC26xx family, unlisted PG nibble
])
def test_cc26xx_unknown_pg_revision_does_not_crash(wafer_bytes):
    CC26xx(_FakeCommandInterface(wafer_bytes))


# --------------------------------------------------------------------------
# Review regressions
# --------------------------------------------------------------------------

def test_no_operation_without_port_does_not_touch_device(monkeypatch):
    cmd = _FakeCmd()
    monkeypatch.setattr(bsl, "CommandInterface", lambda: cmd)
    monkeypatch.setattr(sys, "argv", ["cc2538-bsl"])
    with pytest.raises(SystemExit) as exc:
        bsl.main_cli()
    assert exc.value.code == 2
    assert cmd.calls == []


def test_explicit_port_identifies_target(fake_cli):
    fake_cli()
    assert fake_cli.cmd.calls == ["open", "reset"]


def test_rejected_image_resets_target(fake_cli, tmp_path):
    fw = tmp_path / "big.bin"
    fw.write_bytes(b"\x01" * 0x1004)
    with pytest.raises(SystemExit):
        fake_cli("-e", "-w", str(fw))
    assert fake_cli.cmd.calls == ["open", "reset"]


def test_reset_failure_keeps_original_error(fake_cli, monkeypatch, tmp_path):
    def fail(self):
        raise bsl.CmdException("reset failed")
    monkeypatch.setattr(_FakeCmd, "cmdReset", fail)
    fw = tmp_path / "big.bin"
    fw.write_bytes(b"\x01" * 0x1004)
    with pytest.raises(SystemExit) as exc:
        fake_cli("-w", str(fw))
    assert "past the end of flash" in str(exc.value.code)


def test_default_read_from_unaligned_address(fake_cli, tmp_path):
    out = tmp_path / "out.bin"
    fake_cli("-r", "-a", "1025", str(out))
    assert out.stat().st_size == (0x1000 - 1025) & ~3


def test_unaligned_firmware_size_rejected_before_device(fake_cli, tmp_path):
    fw = tmp_path / "fw.bin"
    fw.write_bytes(b"\x01" * 17)
    with pytest.raises(SystemExit) as exc:
        fake_cli("-e", "-w", str(fw))
    assert "multiple of 4" in str(exc.value.code)
    assert fake_cli.cmd.calls == []


@pytest.mark.parametrize("failing", ["cmdDownload", "cmdSendData"])
def test_write_memory_stops_on_failed_packet(monkeypatch, failing):
    ci = CommandInterface()
    calls = []

    def result(name):
        def call(*args):
            calls.append(name)
            return 0 if name == failing and calls.count(name) == 2 else 1
        return call

    monkeypatch.setattr(ci, "cmdDownload", result("cmdDownload"))
    monkeypatch.setattr(ci, "cmdSendData", result("cmdSendData"))
    data = bytearray(b"\x01" * (248 * 3))
    assert not ci.writeMemory(0, data)
    assert calls.count(failing) == 2


def test_erase_write_covers_every_page(fake_cli, tmp_path):
    fw = tmp_path / "fw.bin"
    fw.write_bytes(b"\x01" * 0x100)
    fake_cli("-W", "-a", str(0x180), str(fw))
    assert fake_cli.cmd.erase_args == (0x100, 0x200)


def test_erase_write_outside_flash_rejected(fake_cli, tmp_path):
    fw = tmp_path / "fw.bin"
    fw.write_bytes(b"\x01" * 0x100)
    with pytest.raises(SystemExit) as exc:
        fake_cli("-W", "-a", str(0x2000), str(fw))
    assert "outside flash" in str(exc.value.code)
    assert "erase_range" not in fake_cli.cmd.calls


def test_invalid_erase_range_rejected_before_erase(fake_cli):
    with pytest.raises(SystemExit):
        fake_cli("-f", "-e", "-E", "a,1")
    assert fake_cli.cmd.calls == ["open", "reset"]


def test_page_prefix_long_form(fake_cli):
    fake_cli("-E", "page,1,3")
    assert fake_cli.cmd.erase_args == (0x100, 0x200)


def test_invalid_ieee_rejected_before_device(fake_cli, tmp_path):
    fw = tmp_path / "fw.bin"
    fw.write_bytes(b"\x01" * 16)
    with pytest.raises(SystemExit):
        fake_cli("-e", "-w", "-i", "nothex", str(fw))
    assert fake_cli.cmd.calls == []


def test_explicit_address_zero_is_kept(fake_cli, monkeypatch, tmp_path):
    class HighFlash(_FakeDevice):
        def __init__(self, command_interface):
            super().__init__(command_interface)
            self.flash_start_addr = 0x00200000

    monkeypatch.setattr(bsl, "CC26xx", HighFlash)
    fake_cli("-r", "-a", "0", "-l", "16", str(tmp_path / "out.bin"))
    assert fake_cli.cmd.read_addr == 12


@pytest.mark.parametrize("size_code, expected", [
    (2, 0x0023FFCC),
    (4, 0x0027FFCC),
])
def test_cc2538_secondary_ieee_follows_flash_size(size_code, expected):
    class Fake:
        def cmdMemRead(self, addr):
            return bytearray([0, 0, 0, size_code << 4])

    assert CC2538(Fake()).addr_ieee_address_secondary == expected


def test_version_falls_back_to_this_checkout(monkeypatch):
    import importlib.metadata

    def missing(name):
        raise importlib.metadata.PackageNotFoundError(name)

    seen = []

    def describe(command, **kwargs):
        seen.append(command)
        return b"2.1-5-gabc\n"

    monkeypatch.setattr(importlib.metadata, "version", missing)
    monkeypatch.setattr(bsl.subprocess, "check_output", describe)
    assert bsl.version() == "2.1-5-gabc"
    assert seen[0][:3] == ["git", "-C", REPO_ROOT]
