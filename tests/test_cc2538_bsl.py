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
import shutil
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


@pytest.mark.skipif(shutil.which("git") is None, reason="needs git")
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
        self.bootloader_address = 0x1000 - 40
        self.bootloader_dis_val = 0

    def ccfg_range(self):
        return 0x1000 - 44, 44

    def _bootloader_enabled(self, word):
        return word != 0

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
                        lambda question, default, *a: questions.append(question) or True)
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


def _packet_interface(monkeypatch, failing_call):
    ci = CommandInterface()
    calls = []

    def result(name):
        def call(*args):
            calls.append(name)
            return 0 if (name, calls.count(name)) == failing_call else 1
        return call

    monkeypatch.setattr(ci, "cmdDownload", result("cmdDownload"))
    monkeypatch.setattr(ci, "cmdSendData", result("cmdSendData"))
    return ci, calls


@pytest.mark.parametrize("failing_call", [
    ("cmdDownload", 1),
    ("cmdSendData", 1),
    ("cmdSendData", 2),
    ("cmdSendData", 3),
])
def test_write_memory_stops_on_failed_packet(monkeypatch, failing_call):
    ci, calls = _packet_interface(monkeypatch, failing_call)
    assert not ci.writeMemory(0, bytearray(b"\x01" * (248 * 3)))
    assert calls.count(failing_call[0]) == failing_call[1]


def test_write_memory_ignores_redundant_final_download(monkeypatch):
    ci, calls = _packet_interface(monkeypatch, ("cmdDownload", 2))
    assert ci.writeMemory(0, bytearray(b"\x01" * (248 * 3)))
    assert calls == ["cmdDownload", "cmdSendData", "cmdSendData",
                     "cmdDownload", "cmdSendData"]


def test_write_memory_checks_final_download_that_starts_transfer(monkeypatch):
    ci, calls = _packet_interface(monkeypatch, ("cmdDownload", 1))
    assert not ci.writeMemory(0, bytearray(b"\xFF" * 248 + b"\x01" * 8))
    assert calls == ["cmdDownload"]


def test_erase_write_covers_every_page(fake_cli, tmp_path):
    fw = tmp_path / "fw.bin"
    fw.write_bytes(b"\x01" * 0x100)
    fake_cli("-f", "-W", "-a", str(0x180), str(fw))
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


def _fake_checkout(monkeypatch, root, project):
    package = root / "cc2538_bsl"
    package.mkdir()
    (root / ".git").mkdir()
    (root / "pyproject.toml").write_text('[project]\nname = "%s"\n' % project)
    monkeypatch.setattr(bsl, "__file__", str(package / "cc2538_bsl.py"))

    import importlib.metadata

    def missing(name):
        raise importlib.metadata.PackageNotFoundError(name)

    monkeypatch.setattr(importlib.metadata, "version", missing)
    seen = []

    def describe(command, **kwargs):
        seen.append(command)
        return b"2.1-5-gabc\n"

    monkeypatch.setattr(bsl.subprocess, "check_output", describe)
    return seen


def test_version_falls_back_to_this_checkout(monkeypatch, tmp_path):
    seen = _fake_checkout(monkeypatch, tmp_path, "cc2538-bsl")
    assert bsl.version() == "2.1-5-gabc"
    assert seen[0][:3] == ["git", "-C", str(tmp_path)]


def test_version_ignores_repo_vendoring_this_package(monkeypatch, tmp_path):
    seen = _fake_checkout(monkeypatch, tmp_path, "host-project")
    assert bsl.version() == bsl.__version__
    assert seen == []


def test_write_outside_flash_rejected_before_erase(fake_cli, monkeypatch, tmp_path):
    class HighFlash(_FakeDevice):
        def __init__(self, command_interface):
            super().__init__(command_interface)
            self.flash_start_addr = 0x00200000

    monkeypatch.setattr(bsl, "CC26xx", HighFlash)
    fw = tmp_path / "fw.bin"
    fw.write_bytes(b"\x01" * 16)
    with pytest.raises(SystemExit) as exc:
        fake_cli("-e", "-w", "-a", "0", str(fw))
    assert "outside flash" in str(exc.value.code)
    assert fake_cli.cmd.calls == ["open", "reset"]


def test_write_outside_flash_allowed_with_force(fake_cli, tmp_path):
    fw = tmp_path / "fw.bin"
    fw.write_bytes(b"\x01" * 16)
    fake_cli("-f", "-w", "-a", str(0x2000), str(fw))
    assert fake_cli.cmd.write_addr == 0x2000


def test_erase_write_outside_flash_rejected_with_force(fake_cli, tmp_path):
    fw = tmp_path / "fw.bin"
    fw.write_bytes(b"\x01" * 16)
    with pytest.raises(SystemExit):
        fake_cli("-f", "-W", "-a", str(0x2000), str(fw))
    assert "erase_range" not in fake_cli.cmd.calls


@pytest.mark.parametrize("flag", ["-w", "-W"])
def test_unaligned_write_address_rejected_before_erase(fake_cli, tmp_path, flag):
    fw = tmp_path / "fw.bin"
    fw.write_bytes(b"\x01" * 16)
    with pytest.raises(SystemExit) as exc:
        fake_cli("-e", flag, "-a", "2", str(fw))
    assert "aligned" in str(exc.value.code)
    assert fake_cli.cmd.calls == ["open", "reset"]


@pytest.mark.parametrize("value", [
    "1ffffffffffffffff",
    "-1",
    "100:22:33:44:55:66:77:88",
])
def test_parse_ieee_address_out_of_range(value):
    with pytest.raises(ValueError):
        parse_ieee_address(value)


def test_file_without_operation_needs_port(monkeypatch, tmp_path):
    cmd = _FakeCmd()
    monkeypatch.setattr(bsl, "CommandInterface", lambda: cmd)
    monkeypatch.setattr(sys, "argv", ["cc2538-bsl", str(tmp_path / "fw.bin")])
    with pytest.raises(SystemExit) as exc:
        bsl.main_cli()
    assert exc.value.code == 2
    assert cmd.calls == []


@pytest.mark.parametrize("erase_range", ["p,4,1", "p,1,1", "p,0,100", "a,0x2000"])
def test_invalid_erase_range_bounds(fake_cli, erase_range):
    with pytest.raises(SystemExit):
        fake_cli("-e", "-E", erase_range)
    assert fake_cli.cmd.calls == ["open", "reset"]


def test_unaligned_decimal_erase_address_is_not_read_as_hex(fake_cli):
    with pytest.raises(SystemExit) as exc:
        fake_cli("-E", "a,1000")
    assert "aligned" in str(exc.value.code)
    assert "erase_range" not in fake_cli.cmd.calls


def test_erase_address_forms(fake_cli):
    # Mixed 0x-prefixed hex and decimal: 0x100 up to 512 (0x200)
    fake_cli("-E", "a,0x100,512")
    assert fake_cli.cmd.erase_args == (0x100, 0x100)


def test_crc_mismatch_survives_failed_reset(fake_cli, monkeypatch, tmp_path):
    def fail(self):
        raise bsl.CmdException("reset failed")
    monkeypatch.setattr(_FakeCmd, "cmdReset", fail)
    monkeypatch.setattr(_FakeDevice, "crc", lambda self, addr, size: 0, raising=False)
    fw = tmp_path / "fw.bin"
    fw.write_bytes(b"\x01" * 16)
    with pytest.raises(SystemExit) as exc:
        fake_cli("-v", str(fw))
    assert "NO CRC32 match" in str(exc.value.code)


def test_failed_read_keeps_existing_output(fake_cli, monkeypatch, tmp_path):
    def fail(self, addr):
        if addr >= 8:
            raise bsl.CmdException("Read failed")
        return bytearray(4)
    monkeypatch.setattr(_FakeDevice, "read_memory", fail)
    out = tmp_path / "out.bin"
    out.write_bytes(b"keep")
    with pytest.raises(SystemExit):
        fake_cli("-r", "-l", "16", str(out))
    assert out.read_bytes() == b"keep"


@pytest.mark.parametrize("cls, method", [
    (CC2538, "cmdMemRead"),
    (CC26xx, "cmdMemReadCC26xx"),
])
def test_read_memory_reports_failed_read(cls, method):
    device = cls.__new__(cls)

    class Fake:
        pass

    fake = Fake()
    setattr(fake, method, lambda addr: None)
    device.command_interface = fake
    with pytest.raises(bsl.CmdException):
        device.read_memory(0x1000)


def test_empty_port_does_not_auto_select(monkeypatch):
    cmd = _FakeCmd()
    monkeypatch.setattr(bsl, "CommandInterface", lambda: cmd)
    monkeypatch.setattr(sys, "argv", ["cc2538-bsl", "-p", ""])
    with pytest.raises(SystemExit) as exc:
        bsl.main_cli()
    assert exc.value.code == 2
    assert cmd.calls == []


def test_checkout_detection_survives_undecodable_pyproject(tmp_path):
    (tmp_path / ".git").mkdir()
    (tmp_path / "pyproject.toml").write_bytes(b'# \xff\nname = "cc2538-bsl"\n')
    assert bsl._is_own_checkout(str(tmp_path))


def test_version_not_resolved_for_normal_commands(fake_cli, monkeypatch):
    def fail():
        raise AssertionError("version() must be lazy")
    monkeypatch.setattr(bsl, "version", fail)
    fake_cli("-e")


def test_read_output_location_checked_before_device(fake_cli, tmp_path):
    with pytest.raises(SystemExit) as exc:
        fake_cli("-r", str(tmp_path / "missing" / "out.bin"))
    assert "Cannot write output file" in str(exc.value.code)
    assert fake_cli.cmd.calls == []


def test_read_output_name_checked_before_device(fake_cli, tmp_path):
    with pytest.raises(SystemExit) as exc:
        fake_cli("-r", str(tmp_path / ("x" * 300)))
    assert "Cannot write output file" in str(exc.value.code)
    assert fake_cli.cmd.calls == []


def test_read_preflight_leaves_no_file_on_failed_read(fake_cli, monkeypatch, tmp_path):
    def fail(self, addr):
        raise bsl.CmdException("Read failed")
    monkeypatch.setattr(_FakeDevice, "read_memory", fail)
    out = tmp_path / "out.bin"
    with pytest.raises(SystemExit):
        fake_cli("-r", "-l", "16", str(out))
    assert not out.exists()
    assert fake_cli.cmd.calls == ["open", "reset"]


@pytest.mark.skipif(not hasattr(os, "geteuid") or os.geteuid() == 0,
                    reason="needs POSIX directory modes as non-root")
def test_writable_output_in_readonly_directory(fake_cli, tmp_path):
    directory = tmp_path / "ro"
    directory.mkdir()
    out = directory / "out.bin"
    out.write_bytes(b"old")
    directory.chmod(0o555)
    try:
        fake_cli("-r", "-l", "16", str(out))
        assert out.stat().st_size == 16
    finally:
        directory.chmod(0o755)


def test_read_output_keeps_hard_links(fake_cli, tmp_path):
    out = tmp_path / "out.bin"
    out.write_bytes(b"old")
    link = tmp_path / "link.bin"
    os.link(str(out), str(link))
    fake_cli("-r", "-l", "16", str(out))
    assert link.stat().st_size == 16


def test_read_output_keeps_mode_and_symlink(fake_cli, tmp_path):
    target = tmp_path / "target.bin"
    target.write_bytes(b"old")
    target.chmod(0o640)
    link = tmp_path / "link.bin"
    link.symlink_to(target)
    fake_cli("-r", "-l", "16", str(link))
    assert link.is_symlink()
    assert target.stat().st_size == 16
    assert target.stat().st_mode & 0o777 == 0o640


def _locked_image():
    image = bytearray(b"\x01" * 0x1000)
    image[0x1000 - 40:0x1000 - 36] = b"\x00\x00\x00\x00"
    return image


def test_declined_backdoor_warning_happens_before_erase(fake_cli, monkeypatch, tmp_path):
    monkeypatch.setattr(bsl, "query_yes_no", lambda question, default, *a: False)
    fw = tmp_path / "fw.bin"
    fw.write_bytes(_locked_image())
    with pytest.raises(SystemExit) as exc:
        fake_cli("-e", "-w", str(fw))
    assert "Aborted" in str(exc.value.code)
    assert fake_cli.cmd.calls == ["open", "reset"]


def test_force_skips_backdoor_warning(fake_cli, monkeypatch, tmp_path):
    def fail(question, default, *a):
        raise AssertionError("must not prompt")
    monkeypatch.setattr(bsl, "query_yes_no", fail)
    fw = tmp_path / "fw.bin"
    fw.write_bytes(_locked_image())
    fake_cli("-f", "-e", "-w", str(fw))
    assert "write" in fake_cli.cmd.calls


@pytest.mark.parametrize("cls, word, disabled", [
    (CC2538, 0xFFFFFFFF, False),
    (CC2538, 0xEFFFFFFF, True),
    (CC26xx, 0xC5FE0AC5, False),
    (CC26xx, 0x00000000, True),
    (CC26xx, 0xC5FFFFFF, True),
])
def test_image_disables_bootloader(cls, word, disabled):
    device = cls.__new__(cls)
    device.bootloader_address = 0x100
    image = bytearray(0x100) + bytearray(word.to_bytes(4, "little"))
    assert device.image_disables_bootloader(image, 0, bsl.Chip.BLANK_WORD) is disabled


def test_erased_cc26xx_ccfg_does_not_warn():
    device = CC26xx.__new__(CC26xx)
    device.bootloader_address = 0x130
    assert device.image_disables_bootloader(
        bytearray(0x100) + bytearray(b"\xFF" * 88), 0, bsl.Chip.BLANK_WORD) is False


def test_image_without_ccfg_does_not_warn():
    device = CC26xx.__new__(CC26xx)
    device.bootloader_address = 0x100
    assert device.image_disables_bootloader(bytearray(0x80), 0, bsl.Chip.BLANK_WORD) is False


def test_empty_erase_range_rejected(fake_cli, tmp_path):
    fw = tmp_path / "fw.bin"
    fw.write_bytes(b"\x01" * 16)
    with pytest.raises(SystemExit):
        fake_cli("-E", "", "-w", str(fw))
    assert fake_cli.cmd.calls == ["open", "reset"]


def test_failed_crc_command_is_reported(fake_cli, monkeypatch, tmp_path):
    monkeypatch.setattr(_FakeDevice, "crc", lambda self, addr, size: None,
                        raising=False)
    fw = tmp_path / "fw.bin"
    fw.write_bytes(b"\x01" * 16)
    with pytest.raises(SystemExit) as exc:
        fake_cli("-v", str(fw))
    assert "CRC read failed" in str(exc.value.code)
    assert fake_cli.cmd.calls[-1] == "reset"


def test_failed_write_stays_in_bootloader(fake_cli, monkeypatch, tmp_path):
    monkeypatch.setattr(_FakeCmd, "writeMemory",
                        lambda self, addr, data, force=False: 0)
    fw = tmp_path / "fw.bin"
    fw.write_bytes(b"\x01" * 16)
    with pytest.raises(SystemExit):
        fake_cli("-w", str(fw))
    assert fake_cli.cmd.calls == ["open"]


def test_interrupt_before_flash_change_resets_target(fake_cli, monkeypatch, tmp_path):
    def interrupt(question, default, *a):
        raise KeyboardInterrupt
    monkeypatch.setattr(bsl, "query_yes_no", interrupt)
    fw = tmp_path / "fw.bin"
    fw.write_bytes(_locked_image())
    with pytest.raises(SystemExit) as exc:
        fake_cli("-e", "-w", str(fw))
    assert "Interrupted" in str(exc.value.code)
    assert fake_cli.cmd.calls == ["open", "reset"]


def test_interrupt_during_write_stays_in_bootloader(fake_cli, monkeypatch, tmp_path):
    def interrupt(self, addr, data, force=False):
        raise KeyboardInterrupt
    monkeypatch.setattr(_FakeCmd, "writeMemory", interrupt)
    fw = tmp_path / "fw.bin"
    fw.write_bytes(b"\x01" * 16)
    with pytest.raises(SystemExit):
        fake_cli("-w", str(fw))
    assert fake_cli.cmd.calls == ["open"]


def test_empty_port_rejected_for_destructive_operation(monkeypatch):
    cmd = _FakeCmd()
    monkeypatch.setattr(bsl, "CommandInterface", lambda: cmd)
    monkeypatch.setattr(sys, "argv", ["cc2538-bsl", "-p", "", "-e"])
    with pytest.raises(SystemExit) as exc:
        bsl.main_cli()
    assert exc.value.code == 2
    assert cmd.calls == []


def test_write_memory_keeps_force_parameter(monkeypatch):
    ci, calls = _packet_interface(monkeypatch, None)
    assert ci.writeMemory(0, bytearray(b"\x01" * 8), force=True)


def test_failed_final_reset_is_not_retried(fake_cli, monkeypatch):
    resets = []

    def fail(self):
        resets.append(1)
        raise bsl.CmdException("reset failed")

    monkeypatch.setattr(_FakeCmd, "cmdReset", fail)
    with pytest.raises(SystemExit) as exc:
        fake_cli("-e")
    assert "reset failed" in str(exc.value.code)
    assert len(resets) == 1


def _crc_mismatch(monkeypatch):
    monkeypatch.setattr(_FakeDevice, "crc", lambda self, addr, size: 0,
                        raising=False)


def test_crc_mismatch_after_write_stays_in_bootloader(fake_cli, monkeypatch, tmp_path):
    _crc_mismatch(monkeypatch)
    fw = tmp_path / "fw.bin"
    fw.write_bytes(b"\x01" * 16)
    with pytest.raises(SystemExit) as exc:
        fake_cli("-w", "-v", str(fw))
    assert "NO CRC32 match" in str(exc.value.code)
    assert fake_cli.cmd.calls == ["open", "write"]


def test_crc_mismatch_without_write_resets(fake_cli, monkeypatch, tmp_path):
    _crc_mismatch(monkeypatch)
    fw = tmp_path / "fw.bin"
    fw.write_bytes(b"\x01" * 16)
    with pytest.raises(SystemExit):
        fake_cli("-v", str(fw))
    assert fake_cli.cmd.calls == ["open", "reset"]


def test_declined_disable_bootloader_resets_before_erase(fake_cli, monkeypatch):
    monkeypatch.setattr(bsl, "query_yes_no", lambda question, default, *a: False)
    with pytest.raises(SystemExit):
        fake_cli("-e", "-D")
    assert fake_cli.cmd.calls == ["open", "reset"]


def test_disable_bootloader_prompts_once(fake_cli, monkeypatch):
    questions = []
    monkeypatch.setattr(bsl, "query_yes_no",
                        lambda question, default, *a: questions.append(question) or True)
    fake_cli("-D")
    assert len(questions) == 1
    assert fake_cli.cmd.calls == ["open", "write", "reset"]


def test_failed_read_with_pending_disable_resets(fake_cli, monkeypatch, tmp_path):
    def fail(self, addr):
        raise bsl.CmdException("Read failed")
    monkeypatch.setattr(_FakeDevice, "read_memory", fail)
    with pytest.raises(SystemExit):
        fake_cli("-D", "-f", "-r", "-l", "16", str(tmp_path / "out.bin"))
    assert fake_cli.cmd.calls == ["open", "reset"]


def test_failed_verify_with_pending_ieee_resets(fake_cli, monkeypatch, tmp_path):
    monkeypatch.setattr(_FakeDevice, "crc", lambda self, addr, size: None,
                        raising=False)
    fw = tmp_path / "fw.bin"
    fw.write_bytes(b"\x01" * 16)
    with pytest.raises(SystemExit):
        fake_cli("-i", "00:12:4B:00:00:00:00:01", "-v", str(fw))
    assert fake_cli.cmd.calls == ["open", "reset"]


def test_read_to_device_node_stays_a_device(fake_cli):
    fake_cli("-r", "-l", "16", os.devnull)
    assert not os.path.isfile(os.devnull)


def test_read_through_dangling_symlink(fake_cli, tmp_path):
    target = tmp_path / "missing.bin"
    link = tmp_path / "link.bin"
    link.symlink_to(target)
    fake_cli("-r", "-l", "16", str(link))
    assert link.is_symlink()
    assert target.stat().st_size == 16


def test_partial_blank_ccfg_does_not_warn():
    device = CC26xx.__new__(CC26xx)
    device.bootloader_address = 0x130
    image = bytearray(0x100) + bytearray(b"\xFF" * 0x40)
    assert device.image_disables_bootloader(image, 0, bsl.Chip.BLANK_WORD) is False


def test_zero_length_read_keeps_output(fake_cli, tmp_path):
    out = tmp_path / "out.bin"
    out.write_bytes(b"keep")
    with pytest.raises(SystemExit):
        fake_cli("-r", "-l", "0", str(out))
    assert out.read_bytes() == b"keep"
    assert fake_cli.cmd.calls == []


@pytest.mark.parametrize("addr, prompted", [(0x00200000, True), (0, False)])
def test_write_memory_guard_only_for_cc2538_image(monkeypatch, addr, prompted):
    ci, _ = _packet_interface(monkeypatch, None)
    questions = []
    monkeypatch.setattr(bsl, "query_yes_no",
                        lambda question, default, *a: questions.append(question) or True)
    image = bytearray(b"\x01" * 524288)
    image[524247] = 0xEF
    assert ci.writeMemory(addr, image)
    assert bool(questions) is prompted


def test_read_to_pipe(fake_cli):
    read_fd, write_fd = os.pipe()
    try:
        fake_cli("-r", "-l", "16", "/dev/fd/%d" % write_fd)
        assert os.read(read_fd, 32) == b"\xAA\xBB\xCC\xDD" * 4
    finally:
        os.close(read_fd)
        os.close(write_fd)


_BL_ENABLED = 0xC5FE0AC5


def _ccfg_tail_device(chip_word=_BL_ENABLED, ccfg_fill=0xFF):
    device = CC26xx.__new__(CC26xx)
    device.bootloader_address = 0x130  # CCFG at 0x100-0x158
    device.reads = []

    def read_memory(addr):
        device.reads.append(addr)
        if addr == 0x130:
            return bytearray(chip_word.to_bytes(4, "little"))
        return bytearray([ccfg_fill] * 4)

    device.read_memory = read_memory
    return device


def test_blank_ccfg_tail_does_not_warn():
    image = bytearray(b"\xFF" * 0x10)
    assert _ccfg_tail_device().image_disables_bootloader(
        image, 0x134, bsl.Chip.BLANK_WORD) is False


def test_app_image_before_ccfg_does_not_warn_on_mass_erase():
    assert _ccfg_tail_device().image_disables_bootloader(
        bytearray(0x80), 0, bsl.Chip.BLANK_WORD) is False


def test_unaligned_erase_write_rejected_before_erase(fake_cli, tmp_path):
    fw = tmp_path / "fw.bin"
    fw.write_bytes(b"\x01" * 16)
    with pytest.raises(SystemExit) as exc:
        fake_cli("-W", "-a", str(0x180), str(fw))
    assert "not aligned" in str(exc.value.code)
    assert fake_cli.cmd.calls == ["open", "reset"]


def test_forced_erase_write_outside_flash_has_no_force_hint(fake_cli, tmp_path):
    fw = tmp_path / "fw.bin"
    fw.write_bytes(b"\x01" * 16)
    with pytest.raises(SystemExit) as exc:
        fake_cli("-f", "-W", "-a", str(0x2000), str(fw))
    assert "--force" not in str(exc.value.code)


def test_erase_write_restoring_ccfg_does_not_prompt(fake_cli, monkeypatch, tmp_path):
    def fail(question, default, *a):
        raise AssertionError("must not prompt")
    monkeypatch.setattr(bsl, "query_yes_no", fail)
    fw = tmp_path / "fw.bin"
    fw.write_bytes(b"\x01" * 0x100)
    fake_cli("-W", "-a", str(0xF00), str(fw))
    assert "write" in fake_cli.cmd.calls


@pytest.mark.parametrize("value", ["a,a00", "a,0020a000"])
def test_unprefixed_hex_erase_address_rejected(fake_cli, value):
    with pytest.raises(SystemExit):
        fake_cli("-E", value)
    assert "erase_range" not in fake_cli.cmd.calls


def test_interrupt_during_reset_is_contained():
    class Cmd:
        def cmdReset(self):
            raise KeyboardInterrupt

    bsl.reset_target(Cmd())


# Without an erase the word on the chip is kept, so it decides the result
@pytest.mark.parametrize("erased, chip_word, disabled", [
    (True, _BL_ENABLED, True),
    (False, _BL_ENABLED, False),
    (False, 0xFFFFFFFF, True),
], ids=["erased", "chip-enabled", "chip-blank"])
def test_ccfg_tail_without_bl_config(erased, chip_word, disabled):
    device = _ccfg_tail_device(chip_word)
    start = bsl.Chip.BLANK_WORD if erased else bytes(device.read_memory(device.bootloader_address))
    assert device.image_disables_bootloader(
        bytearray(0x10), 0x134, start) is disabled


@pytest.mark.parametrize("erased, chip_word, disabled", [
    (True, _BL_ENABLED, True),
    (False, _BL_ENABLED, False),
    (False, 0xFFFFFFFF, True),
], ids=["erased", "chip-enabled", "chip-blank"])
def test_blank_bl_config_in_ccfg(erased, chip_word, disabled):
    image = bytearray(0x100) + bytearray(88)
    image[0x130:0x134] = b"\xFF" * 4
    device = _ccfg_tail_device(chip_word)
    start = bsl.Chip.BLANK_WORD if erased else bytes(device.read_memory(device.bootloader_address))
    assert device.image_disables_bootloader(image, 0, start) is disabled


@pytest.mark.parametrize("end, disabled", [
    (0x131, True),   # writes BL_ENABLE = 0x00
    (0x130, False),  # stops before the word
])
def test_partial_bl_config_bytes_count(end, disabled):
    image = bytearray(end - 0x100)
    device = _ccfg_tail_device()
    assert device.image_disables_bootloader(
        image, 0x100, bytes(device.read_memory(device.bootloader_address))) is disabled


@pytest.mark.parametrize("erase_ranges, reads", [
    ([], list(range(0x100, 0x158, 4))),  # no erase: chip CCFG decides
    ([(0, 0x200)], []),                  # erase: CCFG starts blank
], ids=["no-erase", "erase"])
def test_question_reads_chip_only_without_erase(erase_ranges, reads):
    device = _ccfg_tail_device()
    bsl.ccfg_write_question(device, bytearray(0x10), 0x134, erase_ranges, None)
    assert device.reads == reads


def test_app_image_does_not_read_chip():
    device = _ccfg_tail_device()
    assert bsl.ccfg_write_question(device, bytearray(0x80), 0, [], None) is None
    assert device.reads == []


def test_changed_backdoor_pin_without_erase_asks():
    device = _ccfg_tail_device(0xC5FE0DC5)  # pin 13 on the chip
    image = bytearray(0x100) + bytearray(b"\xFF" * 88)
    image[0x130:0x134] = (0xC5FE0FC5).to_bytes(4, "little")  # pin 15
    question = bsl.ccfg_write_question(device, image, 0, [], None)
    assert "backdoor pin or level will differ" in question
    assert "-E range" in question


def test_same_backdoor_pin_without_erase_does_not_ask():
    device = _ccfg_tail_device(0xC5FE0FC5)
    image = bytearray(0x100) + bytearray(b"\xFF" * 88)
    image[0x130:0x134] = (0xC5FE0FC5).to_bytes(4, "little")
    assert bsl.ccfg_write_question(device, image, 0, [], None) is None


@pytest.mark.parametrize("result", [
    bsl.CmdException("Read failed at address 0x100"),
    bytearray(2),  # short packet
], ids=["error", "short"])
def test_failed_chip_read_stops(result):
    device = _ccfg_tail_device()

    def read_memory(addr):
        if isinstance(result, Exception):
            raise result
        return result

    device.read_memory = read_memory
    with pytest.raises(bsl.CmdException) as exc:
        bsl.ccfg_write_question(device, bytearray(0x10), 0x134, [], None)
    assert "Could not read CCFG" in str(exc.value)
    assert "--skip-ccfg-check" in str(exc.value)


def test_coding_error_in_chip_read_is_not_hidden():
    device = _ccfg_tail_device()

    def read_memory(addr):
        raise AttributeError("bug")

    device.read_memory = read_memory
    with pytest.raises(AttributeError):
        bsl.ccfg_write_question(device, bytearray(0x10), 0x134, [], None)


def test_cc2538_short_read_raises_cmd_exception():
    device = CC2538.__new__(CC2538)
    device.command_interface = type("Ci", (), {
        "cmdMemRead": staticmethod(lambda addr: bytearray(2))})()
    with pytest.raises(bsl.CmdException):
        device.read_memory(0x100)


def test_blank_image_bytes_over_programmed_ccfg_do_not_ask():
    # e.g. an IEEE address set with -i, and firmware leaving that field blank
    device = _ccfg_tail_device(ccfg_fill=0x00)
    image = bytearray(b"\xFF" * 16)  # CCFG.IEEE_MAC and IEEE_BLE unset
    assert bsl.ccfg_write_question(device, image, 0x120, [], None) is None


def test_blank_image_bytes_over_cleared_protection_bits_ask():
    # e.g. CCFG_PROT or ERASE_CONF bits the firmware wants set
    device = _ccfg_tail_device(ccfg_fill=0x00)
    image = bytearray(b"\xFF" * 4)
    question = bsl.ccfg_write_question(device, image, 0x154, [], None)
    assert "0x00000154" in question


def test_disable_bootloader_ignores_lost_bootloader_bits():
    device = _ccfg_tail_device(0xC5FE0DC5)
    image = bytearray(0x100) + bytearray(b"\xFF" * 88)
    image[0x130:0x134] = (0xC5FE0FC5).to_bytes(4, "little")
    assert bsl.ccfg_write_question(
        device, image, 0, [], None, disabling=True) is None


def test_other_ccfg_bits_lost_without_erase_asks():
    device = _ccfg_tail_device(ccfg_fill=0x00)  # chip CCFG programmed
    image = bytearray(b"\x01" * 4)               # CCFG word at 0x100
    question = bsl.ccfg_write_question(device, image, 0x100, [], None)
    assert "0x00000100" in question
    assert "will not match the firmware" in question


def test_disable_bootloader_skips_lockout_question():
    device = _ccfg_tail_device()
    image = bytearray(0x100) + bytearray(88)  # BL_CONFIG = 0
    assert bsl.ccfg_write_question(
        device, image, 0, [(0, 0x200)], None, disabling=True) is None
    assert "backdoor is not enabled" in bsl.ccfg_write_question(
        device, image, 0, [(0, 0x200)], None)


def test_partial_word_on_blank_chip_does_not_blame_firmware():
    device = _ccfg_tail_device(0xFFFFFFFF)
    image = bytearray(0x100) + bytearray(0x30) + bytearray(b"\xC5")
    question = bsl.ccfg_write_question(device, image, 0, [], None)
    assert "not enabled on the target" in question


def _declining_prompt(monkeypatch):
    questions = []

    def decline(question, default, *a):
        questions.append(question)
        return False

    monkeypatch.setattr(bsl, "query_yes_no", decline)
    return questions


@pytest.mark.parametrize("options", [
    ["-e", "-w"],
    ["-E", "a,0xF00,0x1000", "-w"],
    ["-W"],
    ["-e", "-W"],
])
def test_partial_ccfg_restore_after_erase_prompts(fake_cli, monkeypatch, tmp_path, options):
    questions = _declining_prompt(monkeypatch)
    fw = tmp_path / "fw.bin"
    fw.write_bytes(b"\x01" * 0xE0)  # 0xF00-0xFE0 covers only part of CCFG
    with pytest.raises(SystemExit) as exc:
        fake_cli(*options, "-a", str(0xF00), str(fw))
    assert "Aborted" in str(exc.value.code)
    assert len(questions) == 1
    assert "only restores part" in questions[0]
    assert fake_cli.cmd.calls == ["open", "reset"]


@pytest.mark.parametrize("image", [
    b"\x01" * 0x10,                 # ends before CCFG in the CCFG page
    b"\x01" * 0xD4 + b"\xFF" * 44,  # fills CCFG with 0xFF
], ids=["before-ccfg", "blank-ccfg"])
def test_erase_write_without_ccfg_prompts(fake_cli, monkeypatch, tmp_path, image):
    questions = _declining_prompt(monkeypatch)
    fw = tmp_path / "fw.bin"
    fw.write_bytes(image)
    with pytest.raises(SystemExit) as exc:
        fake_cli("-W", "-a", str(0xF00), str(fw))
    assert "Aborted" in str(exc.value.code)
    assert len(questions) == 1
    assert "-W erase includes the CCFG area" in questions[0]
    assert fake_cli.cmd.calls == ["open", "reset"]


@pytest.mark.parametrize("options", [["-e", "-w"], ["-e", "-W"]])
@pytest.mark.parametrize("address, image", [
    (0, b"\x01" * 0x100),
    (0xF00, b"\x01" * 0x10),
    (0xF00, b"\x01" * 0xD4 + b"\xFF" * 44),
], ids=["first-page", "before-ccfg", "blank-ccfg"])
def test_mass_erase_and_write_without_ccfg_agree(fake_cli, monkeypatch, tmp_path, options, address, image):
    def fail(question, default, *a):
        raise AssertionError("must not prompt")
    monkeypatch.setattr(bsl, "query_yes_no", fail)
    fw = tmp_path / "fw.bin"
    fw.write_bytes(image)
    fake_cli(*options, "-a", str(address), str(fw))
    assert "write" in fake_cli.cmd.calls


def test_plain_write_of_ccfg_tail_without_bl_config_does_not_prompt():
    device = _ccfg_tail_device()
    assert bsl.ccfg_write_question(
        device, bytearray(0x10), 0x134, [], None) is None


def test_plain_write_to_blank_chip_without_bl_config_asks():
    device = _ccfg_tail_device(0xFFFFFFFF)
    question = bsl.ccfg_write_question(
        device, bytearray(0x10), 0x134, [], None)
    assert "not enabled on the target" in question


def test_erase_then_ccfg_tail_without_bl_config_asks_once():
    device = _ccfg_tail_device()
    question = bsl.ccfg_write_question(
        device, bytearray(0x10), 0x134, [(0, 0x200)], None)
    assert "does not fully restore it" in question
    assert "NOT be able to reprogram" in question


@pytest.mark.parametrize("chip_bytes, image_word, disabled", [
    (b"\xFF\xFF\xFF\xFF", 0xFFFFFFFF, False),  # blank chip, blank image
    (b"\xEF\xFF\xFF\xFF", 0xFFFFFFFF, True),   # chip backdoor already off
    (b"\xFF\xFF\xFF\xFF", 0xEFFFFFFF, True),   # image turns it off
], ids=["blank", "chip-off", "image-off"])
def test_cc2538_write_without_erase_uses_chip_word(chip_bytes, image_word, disabled):
    # CC2538 COMMAND_MEMORY_READ returns each word byte reversed
    device = CC2538.__new__(CC2538)
    device.bootloader_address = 0x100
    device.command_interface = type("Ci", (), {
        "cmdMemRead": staticmethod(lambda addr: bytearray(chip_bytes))})()
    image = bytearray(0x100) + bytearray(image_word.to_bytes(4, "little"))
    image += bytearray(40)  # rest of CCFG, so the image programs CCFG
    assert device.image_disables_bootloader(
        image, 0, bytes(device.read_memory(device.bootloader_address))) is disabled


def test_plain_write_with_ccfg_reads_chip_and_asks(fake_cli, monkeypatch, tmp_path):
    questions = _declining_prompt(monkeypatch)
    monkeypatch.setattr(_FakeDevice, "read_memory",
                        lambda self, addr: (self.command_interface.calls.append("read")
                                            or bytearray(4)))
    fw = tmp_path / "fw.bin"
    fw.write_bytes(b"\x01" * 0x100)  # last page, BL word 0x01010101
    with pytest.raises(SystemExit) as exc:
        fake_cli("-w", "-a", str(0xF00), str(fw))
    assert "Aborted" in str(exc.value.code)
    assert "cannot set" in questions[0]
    assert fake_cli.cmd.calls == ["open"] + ["read"] * 11 + ["reset"]


def test_forced_plain_write_does_not_read_chip(fake_cli, monkeypatch, tmp_path):
    def fail(self, addr):
        raise AssertionError("must not read")
    monkeypatch.setattr(_FakeDevice, "read_memory", fail)
    fw = tmp_path / "fw.bin"
    fw.write_bytes(b"\x01" * 0x100)
    fake_cli("-f", "-w", "-a", str(0xF00), str(fw))
    assert "write" in fake_cli.cmd.calls


def test_locked_image_question_names_backdoor():
    device = _ccfg_tail_device()
    image = bytearray(0x100) + bytearray(88)  # BL_CONFIG = 0
    question = bsl.ccfg_write_question(device, image, 0, [], None)
    assert "backdoor is not enabled" in question


def test_closed_input_at_prompt_mentions_force(monkeypatch):
    def closed():
        raise EOFError
    monkeypatch.setattr("builtins.input", closed)
    with pytest.raises(Exception) as exc:
        bsl.query_yes_no("Continue?", "no")
    assert "--force" in str(exc.value)
    assert "--skip-ccfg-check" not in str(exc.value)


def test_interrupted_reset_warning_has_reason(capsys):
    class Cmd:
        def cmdReset(self):
            raise KeyboardInterrupt

    bsl.reset_target(Cmd())
    assert "could not reset target: interrupted" in capsys.readouterr().err


@pytest.mark.parametrize("reads", [[b""], [b"\x06\x00", b"\x00"]],
                         ids=["no-reply", "short-data"])
def test_receive_packet_timeout_raises_cmd_exception(monkeypatch, reads):
    ci = CommandInterface()
    replies = list(reads)
    monkeypatch.setattr(ci, "_read", lambda n: replies.pop(0))
    with pytest.raises(bsl.CmdException):
        ci.receivePacket()


def test_cc26xx_short_read_raises_cmd_exception():
    device = CC26xx.__new__(CC26xx)
    device.command_interface = type("Ci", (), {
        "cmdMemReadCC26xx": staticmethod(lambda addr: bytearray(2))})()
    with pytest.raises(bsl.CmdException):
        device.read_memory(0x100)


def test_blank_bl_config_over_enabled_chip_does_not_ask():
    device = _ccfg_tail_device(0xC5FE0FC5)
    image = bytearray(0x100) + bytearray(b"\xFF" * 88)
    image[0x100:0x104] = b"\x00" * 4  # some other CCFG setting
    assert bsl.ccfg_write_question(device, image, 0, [], None) is None


def _ieee_device(chip_bytes):
    device = _ccfg_tail_device()
    device.addr_ieee_address_secondary = 0x120
    device.page_size = 0x100

    def read_memory(addr):
        device.reads.append(addr)
        return bytearray(chip_bytes[addr - 0x120:addr - 0x120 + 4])

    device.read_memory = read_memory
    return device


NEW_IEEE = (0x00124B0000000002).to_bytes(8, "little")


@pytest.mark.parametrize("chip, asks", [
    (b"\xFF" * 8, False),                                # blank
    ((0x00124B0000000001).to_bytes(8, "little"), True),  # other address
    (NEW_IEEE, False),                                   # same address
], ids=["blank", "other", "same"])
def test_ieee_write_checks_chip(chip, asks):
    device = _ieee_device(chip)
    question = bsl.ieee_write_question(device, NEW_IEEE, None, 0, [])
    assert (question is not None) is asks
    assert device.reads == [0x120, 0x124]


def test_ieee_write_after_erase_does_not_read():
    device = _ieee_device(b"\x00" * 8)
    assert bsl.ieee_write_question(
        device, NEW_IEEE, None, 0, [(0, 0x200)]) is None
    assert device.reads == []


def test_ieee_write_over_image_bytes_asks():
    device = _ieee_device(b"\xFF" * 8)
    image = bytearray(b"\xFF" * 0x120) + bytearray(8)  # image zeroes the field
    assert bsl.ieee_write_question(device, NEW_IEEE, image, 0, []) is not None


def test_cli_ieee_over_other_address_asks(fake_cli, monkeypatch):
    questions = _declining_prompt(monkeypatch)
    monkeypatch.setattr(_FakeDevice, "addr_ieee_address_secondary",
                        0x1000 - 44 - 8, raising=False)
    with pytest.raises(SystemExit) as exc:
        fake_cli("-i", "00:12:4b:00:00:00:00:02")
    assert "Aborted" in str(exc.value.code)
    assert "IEEE address area" in questions[0]
    assert fake_cli.cmd.calls == ["open", "read", "read", "reset"]


def test_partial_image_word_disabling_enabled_target_blames_firmware():
    device = _ccfg_tail_device(0xC5FE0FC5)
    image = bytearray(0x31)  # ends at BL_CONFIG+1 with BL_ENABLE = 0x00
    question = bsl.ccfg_write_question(device, image, 0x100, [], None)
    assert "in the firmware you are about to write disables" in question
    assert "NOT be able to reprogram the target" in question


@pytest.mark.parametrize("image, mentions_e, mentions_w", [
    (None, False, False),
    (bytearray(b"\xFF" * 4), True, False),      # app before the IEEE page
    (bytearray(b"\xFF" * 0x104), True, True),   # reaches the IEEE page
], ids=["ieee-only", "write-elsewhere", "write-reaching-page"])
def test_ieee_hint_matches_run(image, mentions_e, mentions_w):
    device = _ieee_device((0x00124B0000000001).to_bytes(8, "little"))
    question = bsl.ieee_write_question(device, NEW_IEEE, image, 0, [])
    assert "-E range" in question
    assert ("-e" in question) is mentions_e
    assert ("-W" in question) is mentions_w


@pytest.mark.parametrize("size", [0, 1])
def test_receive_packet_rejects_tiny_size(monkeypatch, size):
    ci = CommandInterface()
    replies = [bytes([size, 0])]
    monkeypatch.setattr(ci, "_read", lambda n: replies.pop(0))
    with pytest.raises(bsl.CmdException):
        ci.receivePacket()


@pytest.mark.parametrize("name", ["cmdCRC32", "cmdCRC32CC26xx"])
def test_short_crc_reply_raises_cmd_exception(monkeypatch, name):
    ci = CommandInterface()
    monkeypatch.setattr(ci, "_write", lambda *a: None)
    monkeypatch.setattr(ci, "_wait_for_ack", lambda *a: 1)
    monkeypatch.setattr(ci, "receivePacket", lambda: bytearray(2))
    monkeypatch.setattr(ci, "checkLastCmd", lambda: 1)
    with pytest.raises(bsl.CmdException):
        getattr(ci, name)(0, 4)


def test_forced_erase_write_skips_ccfg_checks(fake_cli, monkeypatch, tmp_path):
    def fail(*args):
        raise AssertionError("must not prompt or read")
    monkeypatch.setattr(bsl, "query_yes_no", fail)
    monkeypatch.setattr(_FakeDevice, "read_memory", fail)
    fw = tmp_path / "fw.bin"
    fw.write_bytes(b"\x01" * 0xE0)  # partial CCFG restore
    fake_cli("-f", "-W", "-a", str(0xF00), str(fw))
    assert "erase_range" in fake_cli.cmd.calls
    assert "write" in fake_cli.cmd.calls


@pytest.mark.parametrize("options", [
    ["-w", "FW"],                     # app that misses the guessed CCFG page
    ["-e", "-w", "FW"],
    ["-W", "-a", str(0xF00), "FW"],   # reaches it
    ["-i", "00:12:4b:00:00:00:00:02"],
    ["-D"],
], ids=["w", "e-w", "W", "i", "D"])
def test_unknown_ccfg_location_asks_once(fake_cli, monkeypatch, tmp_path, options):
    questions = _declining_prompt(monkeypatch)
    monkeypatch.setattr(_FakeDevice, "ccfg_known", False, raising=False)
    monkeypatch.setattr(_FakeDevice, "addr_ieee_address_secondary",
                        0x1000 - 44 - 8, raising=False)
    fw = tmp_path / "fw.bin"
    fw.write_bytes(b"\x01" * 0x100)
    args = [str(fw) if o == "FW" else o for o in options]
    with pytest.raises(SystemExit) as exc:
        fake_cli(*args)
    assert "Aborted" in str(exc.value.code)
    assert len(questions) == 1
    assert "not recognised" in questions[0]
    assert fake_cli.cmd.calls == ["open", "reset"]


def test_unknown_wafer_marks_ccfg_unknown():
    fake = _FakeCommandInterface([0x00, 0x00, 0x00, 0x00])
    assert CC26xx(fake).ccfg_known is False


def test_cc2538_blank_image_word_keeps_chip_backdoor():
    device = CC2538.__new__(CC2538)
    device.bootloader_address = 0x100
    image = bytearray(b"\xFF" * 0x12C)  # padding up to the end of CCFG
    image[0x104:0x108] = b"\x00" * 4   # another CCFG setting
    # Chip word 0xF3FFFFFF (backdoor on PA3), rest of CCFG blank
    device.read_memory = lambda addr: (bytearray(b"\xFF\xFF\xFF\xF3")
                                       if addr == 0x100 else bytearray(b"\xFF" * 4))
    assert bsl.ccfg_write_question(device, image, 0, [], None) is None


def test_image_disables_bootloader_defaults_to_blank_start():
    device = _ccfg_tail_device()
    image = bytearray(0x100) + bytearray(88)  # BL_CONFIG = 0
    assert device.image_disables_bootloader(image, 0) is True


@pytest.mark.parametrize("code, known", [
    (0, False), (1, True), (4, True), (5, False), (7, False),
])
def test_cc2538_flash_size_code_sets_known(code, known):
    class Ci:
        def cmdMemRead(self, addr):
            return bytearray([0x00, 0x00, 0x00, code << 4])

    device = CC2538(Ci())
    assert device.ccfg_known is known
    assert device.size_known is known
    assert device.size_code == (None if known else code)


@pytest.mark.parametrize("options, reaches", [
    (["-e", "-w", "FW"], "write"),
    (["-W", "-a", str(0xF00), "FW"], "write"),
    (["-i", "00:12:4b:00:00:00:00:02"], "write"),
], ids=["e-w", "W", "i"])
def test_unknown_ccfg_location_accepted_goes_ahead(fake_cli, monkeypatch, tmp_path, options, reaches):
    questions = []
    monkeypatch.setattr(bsl, "query_yes_no",
                        lambda q, d, *a: questions.append(q) or True)
    monkeypatch.setattr(_FakeDevice, "ccfg_known", False, raising=False)
    monkeypatch.setattr(_FakeDevice, "addr_ieee_address_secondary",
                        0x1000 - 44 - 8, raising=False)
    fw = tmp_path / "fw.bin"
    fw.write_bytes(b"\x01" * 0x100)
    fake_cli(*[str(fw) if o == "FW" else o for o in options])
    assert len(questions) == 1
    assert reaches in fake_cli.cmd.calls


def test_unknown_ccfg_location_disable_bootloader_asks_both(fake_cli, monkeypatch):
    questions = []
    monkeypatch.setattr(bsl, "query_yes_no",
                        lambda q, d, *a: questions.append(q) or True)
    monkeypatch.setattr(_FakeDevice, "ccfg_known", False, raising=False)
    fake_cli("-D")
    assert "guessed address" in questions[0]
    assert "Disabling the bootloader" in questions[1]


@pytest.mark.parametrize("flag", ["-f", "--skip-ccfg-check"])
def test_unknown_ccfg_location_flags_skip_question(fake_cli, monkeypatch, tmp_path, flag):
    def fail(question, default, *a):
        raise AssertionError("must not prompt")
    monkeypatch.setattr(bsl, "query_yes_no", fail)
    monkeypatch.setattr(_FakeDevice, "ccfg_known", False, raising=False)
    fw = tmp_path / "fw.bin"
    fw.write_bytes(b"\x01" * 0x100)
    fake_cli(flag, "-e", "-w", str(fw))
    assert "write" in fake_cli.cmd.calls


def test_skip_ccfg_check_keeps_bounds_check(fake_cli, monkeypatch, tmp_path):
    fw = tmp_path / "fw.bin"
    fw.write_bytes(b"\x01" * 0x2000)  # larger than the fake flash
    with pytest.raises(SystemExit):
        fake_cli("--skip-ccfg-check", "-e", "-w", str(fw))
    assert "write" not in fake_cli.cmd.calls


def test_skip_ccfg_check_skips_ccfg_question_and_read(fake_cli, monkeypatch, tmp_path):
    def fail(*args):
        raise AssertionError("must not prompt or read")
    monkeypatch.setattr(bsl, "query_yes_no", fail)
    monkeypatch.setattr(_FakeDevice, "read_memory", fail)
    fw = tmp_path / "fw.bin"
    fw.write_bytes(b"\x01" * 0xE0)  # partial CCFG restore after -W
    fake_cli("--skip-ccfg-check", "-W", "-a", str(0xF00), str(fw))
    assert "write" in fake_cli.cmd.calls


def test_cc2538_unknown_size_mass_erase_asks():
    device = _FakeDevice(_FakeCmd())
    device.ccfg_known = False
    device.size_known = False
    args = type("Args", (), {"write": False, "erase_write": False,
                             "disable_bootloader": False, "erase": True})()
    question = bsl.guessed_ccfg_question(device, args, None)
    assert "-e erases only the guessed 4 KB" in question


def test_unknown_size_read_does_not_ask(fake_cli, monkeypatch, tmp_path):
    def fail(question, default, *args):
        raise AssertionError("must not prompt")
    monkeypatch.setattr(bsl, "query_yes_no", fail)
    monkeypatch.setattr(_FakeDevice, "ccfg_known", False, raising=False)
    monkeypatch.setattr(_FakeDevice, "size_known", False, raising=False)
    monkeypatch.setattr(_FakeDevice, "size_code", 0, raising=False)
    out = tmp_path / "out.bin"
    fake_cli("-r", str(out))
    assert out.stat().st_size == 0x1000  # up to the guessed end of flash


def test_closed_input_hint_is_per_question(monkeypatch):
    def closed():
        raise EOFError
    monkeypatch.setattr("builtins.input", closed)
    with pytest.raises(Exception) as exc:
        bsl.query_yes_no("Continue?", "no", bsl.SKIP_CCFG_HINT)
    assert "--skip-ccfg-check" in str(exc.value)


def test_unknown_size_warning_shown_with_quiet(fake_cli, monkeypatch, tmp_path, capsys):
    monkeypatch.setattr(_FakeDevice, "size_known", False, raising=False)
    monkeypatch.setattr(_FakeDevice, "size_code", 5, raising=False)
    fake_cli("-q", "-r", "-l", "16", str(tmp_path / "out.bin"))
    assert "unknown flash size code 5, assuming 4 KB" in capsys.readouterr().err


@pytest.mark.parametrize("address, length, refused", [
    (0x00200000, 0x10000, False),  # inside the guessed 64 KB
    (0x0020F000, 0x2000, True),    # crosses the guessed end
    (0x00210000, 0x1000, True),    # past it, inside the largest flash
    (0x20000000, 0x100, False),    # RAM
])
def test_write_past_guessed_end_refused(address, length, refused):
    device = CC2538.__new__(CC2538)
    device.flash_start_addr = 0x00200000
    device.size = 0x10000
    device.size_known = False
    device.size_code = 5
    if refused:
        with pytest.raises(bsl.CmdException) as exc:
            bsl.check_guessed_flash_end(device, address, length)
        assert "code 5" in str(exc.value)
    else:
        bsl.check_guessed_flash_end(device, address, length)


def test_erase_range_error_names_guessed_size():
    device = _FakeDevice(_FakeCmd())
    device.size_known = False
    device.size_code = 6
    with pytest.raises(ValueError) as exc:
        bsl.check_erase_range(device, 0x1000, 0x100)
    assert "unknown flash size code 6" in str(exc.value)
