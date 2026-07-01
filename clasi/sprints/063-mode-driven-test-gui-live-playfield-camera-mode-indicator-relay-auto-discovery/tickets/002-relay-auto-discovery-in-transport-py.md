---
id: '002'
title: Relay auto-discovery in transport.py
status: open
use-cases:
- SUC-002
depends-on:
- '001'
github-issue: ''
issue: live-camera-view-for-the-test-gui.md
completes_issue: false
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Relay auto-discovery in transport.py

## Description

Add relay auto-discovery so that clicking Connect in Relay mode finds the
relay serial port automatically without the user typing a port number.

Two functions are added to `transport.py`:

1. `find_relay_port(port_list, probe_fn)` — pure, injectable, Qt-free. Given
   a list of port names and a callable `probe_fn(port) -> str | None` that
   returns the device banner, returns the first port whose banner contains
   `"RADIOBRIDGE"`, or `None`.

2. `_relay_probe_banner(port)` — the real I/O probe. Opens the port with DTR
   asserted (per relay protocol: DTR-pulse triggers the reset/announce
   handshake), waits up to ~1 s for a `DEVICE:` line, returns it or `None`
   on timeout or I/O error. Closes the port before returning regardless of
   outcome.

`_on_connect()` in `__main__.py` is updated: when Relay is selected, it calls
`find_relay_port(list_ports(), _relay_probe_banner)`. On success it populates
`port_edit` with the discovered port and proceeds normally. On failure it logs
a clear "[WARN] No relay found on any serial port" and returns without
connecting.

**Critical reference:** The relay `!GO` data-plane protocol and banner format
(`DEVICE:RADIOBRIDGE:relay:gozop:<id>`) are documented in
`.clasi/knowledge/2026-06-12-relay-go-data-plane-and-docs.md` and at
https://robots.jointheleague.org/. Read both before implementing
`_relay_probe_banner`. Key point: open with DTR asserted (pyserial default),
wait ~1 s for the `DEVICE:` announcement line. Do NOT send `!GO` during
banner probing — just read the announcement.

**Files to modify:**
- `host/robot_radio/testgui/transport.py` — add `find_relay_port()` and
  `_relay_probe_banner()`.
- `host/robot_radio/testgui/__main__.py` — update `_on_connect()` relay path.

## Acceptance Criteria

- [ ] `find_relay_port(["portA", "portB"], probe)` returns `"portA"` when
      `probe("portA")` returns a string containing `"RADIOBRIDGE"` and
      `probe("portB")` returns `None`.
- [ ] `find_relay_port([], probe)` returns `None`.
- [ ] `find_relay_port(["portA"], probe)` returns `None` when `probe("portA")`
      returns `None`.
- [ ] `find_relay_port` calls `probe_fn` only until a match is found (stops early).
- [ ] `probe_fn` exceptions (I/O errors) are caught; the port is skipped, not fatal.
- [ ] `_relay_probe_banner(port)` returns `None` on a non-existent port without
      raising (defensive timeout/error handling).
- [ ] In the GUI, clicking Connect in Relay mode with the relay plugged in
      discovers and connects to the relay; `port_edit` is **populated with
      the discovered port** (so the user can see which port was used); the
      log shows "[INFO] Relay found on /dev/...". Both the port field
      update and the log entry are required — not one or the other.
- [ ] In the GUI, clicking Connect in Relay mode with no relay plugged in
      logs "[WARN] No relay found on any serial port" and does not connect.
- [ ] Discovery does not disrupt non-relay serial devices probed along the way
      (no commands sent, port closed cleanly after banner read).
- [ ] All existing `tests/testgui/` tests pass unchanged.

## Implementation Plan

### Approach

#### `transport.py` additions

```python
def find_relay_port(
    port_list: list[str],
    probe_fn: "Callable[[str], str | None]",
) -> "str | None":
    """Return the first port in port_list whose banner contains 'RADIOBRIDGE'.

    Calls probe_fn(port) for each candidate in order; stops at the first match.
    probe_fn exceptions are caught and the port is skipped.
    Returns None if no match found.
    """
    for port in port_list:
        try:
            banner = probe_fn(port)
        except Exception:
            continue
        if banner and "RADIOBRIDGE" in banner:
            return port
    return None


def _relay_probe_banner(port: str, timeout_s: float = 1.2) -> "str | None":
    """Open port with DTR asserted and read the DEVICE: announcement line.

    The relay resets on DTR-pulse (the pyserial default open) and announces:
      DEVICE:RADIOBRIDGE:relay:gozop:<id>
    Returns the banner line or None on timeout/error. Always closes the port.

    Do NOT send any commands. Just read the announcement.
    """
    import serial  # type: ignore[import]
    ser = None
    try:
        ser = serial.Serial(port, 115200, timeout=timeout_s)
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            line = ser.readline().decode("ascii", errors="replace").strip()
            if line.startswith("DEVICE:"):
                return line
        return None
    except Exception:
        return None
    finally:
        if ser is not None:
            try:
                ser.close()
            except Exception:
                pass
```

#### `__main__.py` `_on_connect()` relay path

Replace:
```python
elif name == "Relay":
    if not port:
        _append_log("[ERROR] No port specified for Relay transport")
        return
    transport = RelayTransport(port)
```

With:
```python
elif name == "Relay":
    from robot_radio.testgui.transport import find_relay_port, _relay_probe_banner
    _append_log("[INFO] Relay: scanning serial ports for relay...")
    discovered = find_relay_port(list_ports(), _relay_probe_banner)
    if discovered is None:
        # Fall back to port_edit if user typed one
        discovered = port_edit.text().strip() or None
    if discovered is None:
        _append_log("[WARN] No relay found on any serial port")
        return
    _append_log(f"[INFO] Relay found on {discovered}")
    port_edit.setText(discovered)
    transport = RelayTransport(discovered)
```

### Files to create/modify

- `host/robot_radio/testgui/transport.py`: add `find_relay_port()`,
  `_relay_probe_banner()`.
- `host/robot_radio/testgui/__main__.py`: update relay branch of `_on_connect()`.

### Testing plan

Add `tests/testgui/test_relay_discovery.py` (or extend `test_transport.py`):

```python
# All Qt-free — no QApplication needed.

from robot_radio.testgui.transport import find_relay_port

def test_find_relay_port_match():
    def probe(port):
        return "DEVICE:RADIOBRIDGE:relay:gozop:abc123" if port == "/dev/relay" else None
    assert find_relay_port(["/dev/other", "/dev/relay"], probe) == "/dev/relay"

def test_find_relay_port_no_match():
    assert find_relay_port(["/dev/portA", "/dev/portB"], lambda p: None) is None

def test_find_relay_port_empty_list():
    assert find_relay_port([], lambda p: "DEVICE:RADIOBRIDGE:...") is None

def test_find_relay_port_stops_early():
    calls = []
    def probe(port):
        calls.append(port)
        return "DEVICE:RADIOBRIDGE:relay:gozop:x" if port == "/dev/first" else None
    find_relay_port(["/dev/first", "/dev/second"], probe)
    assert "/dev/second" not in calls

def test_find_relay_port_skips_exception():
    def probe(port):
        if port == "/dev/bad":
            raise IOError("port exploded")
        return "DEVICE:RADIOBRIDGE:relay:gozop:y"
    # Exception on first port; second port matches.
    result = find_relay_port(["/dev/bad", "/dev/good"], probe)
    assert result == "/dev/good"

def test_find_relay_port_no_radiobridge_in_banner():
    assert find_relay_port(["/dev/robot"], lambda p: "DEVICE:NEZHA2:robot:tovez:1") is None
```

### Documentation updates

Update the module docstring in `transport.py` to document `find_relay_port()`
and `_relay_probe_banner()` in the public surface section.
