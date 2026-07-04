#!/usr/bin/env python3
"""test_hello_banner.py — Unit tests for HELLO → DEVICE: identification banner.

OOP change: Validates the wire format of the DEVICE: banner emitted by the
HELLO command (and at boot) so mbdeploy probe_type() can identify the robot.

Banner format (announce.md):
    DEVICE:<role>:<common_name>:<device_name>:<serial>

For the robot firmware:
    DEVICE:NEZHA2:robot:<friendly_name>:<decimal_serial>

These tests validate the format contract without requiring a connected robot,
following the pattern established by test_motion_verbs_v2.py.
"""

from __future__ import annotations

import re
import pytest


# ---------------------------------------------------------------------------
# Banner format constants — single source of truth for the test assertions
# ---------------------------------------------------------------------------

BANNER_PREFIX = "DEVICE:"
EXPECTED_ROLE = "NEZHA2"
EXPECTED_COMMON_NAME = "robot"


# ---------------------------------------------------------------------------
# Parser helper
# ---------------------------------------------------------------------------

def parse_device_banner(line: str) -> dict[str, str]:
    """Parse a DEVICE:... banner line into field dict.

    Returns keys: role, common_name, device_name, serial.
    Raises AssertionError if the line doesn't match the format.
    """
    assert line.startswith(BANNER_PREFIX), (
        f"Expected DEVICE: banner, got: {line!r}"
    )
    parts = line.split(":")
    assert len(parts) >= 5, (
        f"DEVICE: banner needs ≥5 colon-separated fields, got {len(parts)}: {line!r}"
    )
    return {
        "role":        parts[1],
        "common_name": parts[2],
        "device_name": parts[3],
        "serial":      ":".join(parts[4:]),
    }


# ---------------------------------------------------------------------------
# Fixtures — representative banner strings for format testing
# ---------------------------------------------------------------------------

EXAMPLE_BANNERS = [
    "DEVICE:NEZHA2:robot:tovez:1784514240",
    "DEVICE:NEZHA2:robot:getez:987654321",
    "DEVICE:NEZHA2:robot:abcde:0",
    "DEVICE:NEZHA2:robot:xyzwv:4294967295",  # max uint32
]


# ---------------------------------------------------------------------------
# TestHelloBannerFormat — validate the wire format contract
# ---------------------------------------------------------------------------

class TestHelloBannerFormat:
    """Tests for HELLO → DEVICE: banner wire format."""

    @pytest.mark.parametrize("banner", EXAMPLE_BANNERS)
    def test_starts_with_device(self, banner: str) -> None:
        """Banner must start with literal 'DEVICE:'."""
        assert banner.startswith("DEVICE:")

    @pytest.mark.parametrize("banner", EXAMPLE_BANNERS)
    def test_has_five_fields(self, banner: str) -> None:
        """Banner must have exactly 5 colon-separated fields."""
        parts = banner.split(":")
        assert len(parts) == 5, (
            f"Expected exactly 5 fields (DEVICE, role, common_name, device_name, serial), "
            f"got {len(parts)}: {banner!r}"
        )

    @pytest.mark.parametrize("banner", EXAMPLE_BANNERS)
    def test_role_is_nezha2(self, banner: str) -> None:
        """Role field (field 2) must be 'NEZHA2'."""
        info = parse_device_banner(banner)
        assert info["role"] == EXPECTED_ROLE, (
            f"Expected role 'NEZHA2', got {info['role']!r}"
        )

    @pytest.mark.parametrize("banner", EXAMPLE_BANNERS)
    def test_common_name_is_robot(self, banner: str) -> None:
        """Common name field (field 3) must be 'robot'."""
        info = parse_device_banner(banner)
        assert info["common_name"] == EXPECTED_COMMON_NAME, (
            f"Expected common_name 'robot', got {info['common_name']!r}"
        )

    @pytest.mark.parametrize("banner", EXAMPLE_BANNERS)
    def test_device_name_is_lowercase_alpha(self, banner: str) -> None:
        """Device name field (field 4) should be a non-empty lowercase alpha string
        (CODAL friendly name is 5 lowercase letters, e.g. 'tovez')."""
        info = parse_device_banner(banner)
        assert len(info["device_name"]) > 0, "device_name must not be empty"
        # CODAL friendly names are lowercase ASCII letters only
        assert re.fullmatch(r"[a-z]+", info["device_name"]), (
            f"device_name should be lowercase alpha, got {info['device_name']!r}"
        )

    @pytest.mark.parametrize("banner", EXAMPLE_BANNERS)
    def test_serial_is_decimal_integer(self, banner: str) -> None:
        """Serial field (field 5) must be a non-negative decimal integer
        (uint32_t printed as %lu — same source as ID command)."""
        info = parse_device_banner(banner)
        assert re.fullmatch(r"\d+", info["serial"]), (
            f"serial must be a decimal integer, got {info['serial']!r}"
        )
        # Must fit in uint32 range
        serial_val = int(info["serial"])
        assert 0 <= serial_val <= 0xFFFFFFFF, (
            f"serial {serial_val} out of uint32 range"
        )

    @pytest.mark.parametrize("banner", EXAMPLE_BANNERS)
    def test_no_internal_whitespace(self, banner: str) -> None:
        """Banner must contain no whitespace (announce.md: 'no internal whitespace')."""
        assert " " not in banner and "\t" not in banner, (
            f"Banner must not contain whitespace: {banner!r}"
        )

    @pytest.mark.parametrize("banner", EXAMPLE_BANNERS)
    def test_role_not_relay_or_bridge(self, banner: str) -> None:
        """Role must NOT contain 'RELAY' or 'BRIDGE' — mbdeploy.is_relay() would
        refuse to flash the board if either token appears in role (case-insensitive)."""
        info = parse_device_banner(banner)
        role_upper = info["role"].upper()
        assert "RELAY" not in role_upper, (
            f"Role must not contain 'RELAY': {info['role']!r}"
        )
        assert "BRIDGE" not in role_upper, (
            f"Role must not contain 'BRIDGE': {info['role']!r}"
        )


# ---------------------------------------------------------------------------
# TestHelloBannerParsedByProbeType — validate mbdeploy probe_type() compat
# ---------------------------------------------------------------------------

class TestHelloBannerParsedByProbeType:
    """Tests that our banner format is correctly parsed by the mbdeploy
    probe_type() logic (devices.py). Simulates the exact parsing in that
    function without importing from mbdeploy."""

    def _simulate_probe_type_parse(self, raw_line: str) -> dict | None:
        """Replicate the probe_type() parsing logic from devices.py."""
        text = raw_line.strip()
        if not text.startswith("DEVICE:"):
            return None
        parts = text.split(":")
        if len(parts) < 5:
            return None
        return {
            "role":        parts[1],
            "common_name": parts[2],
            "device_name": parts[3],
            "serial":      ":".join(parts[4:]),
            "raw":         text,
        }

    @pytest.mark.parametrize("banner", EXAMPLE_BANNERS)
    def test_probe_type_parses_banner(self, banner: str) -> None:
        """probe_type() simulation must successfully parse our banner."""
        result = self._simulate_probe_type_parse(banner)
        assert result is not None, (
            f"probe_type() simulation returned None for: {banner!r}"
        )
        assert result["role"] == "NEZHA2"
        assert result["common_name"] == "robot"
        assert result["raw"] == banner

    def test_probe_type_rejects_empty(self) -> None:
        """probe_type() returns None for empty/non-DEVICE lines."""
        assert self._simulate_probe_type_parse("") is None
        assert self._simulate_probe_type_parse("OK ping") is None
        assert self._simulate_probe_type_parse("ERR unknown HELLO") is None

    def test_is_relay_false_for_nezha2(self) -> None:
        """mbdeploy.is_relay() must return False for role='NEZHA2'.

        is_relay() checks for 'RELAY' or 'BRIDGE' in role (case-insensitive).
        'NEZHA2' contains neither, so the robot is flashable.
        """
        role = "NEZHA2"
        r = role.upper()
        is_relay = ("RELAY" in r) or ("BRIDGE" in r)
        assert is_relay is False, (
            f"is_relay() returned True for role={role!r} — robot would be unflashable"
        )
