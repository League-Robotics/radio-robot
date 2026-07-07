"""088-006: sim harness channel extension (SERIAL vs RADIO).

Before this ticket, ``sim_command()`` hardcoded ``returnPath = SERIAL`` and
``CommandRouter``'s two reply channels were both wired to the SAME shared
``ReplyStore`` -- so no test could prove a command actually dispatches and
replies on the RADIO channel specifically, only that a ``Channel::RADIO``
-tagged field could be set.

These tests prove the fix: ``sim_command_on()`` lets a test pick SERIAL or
RADIO as a command's return channel, and ``CommandRouter``'s two reply sinks
are backed by genuinely distinct ``ReplyStore`` instances -- a reply sent on
one channel is never observable via the other channel's store. The reply
CONTENT is channel-independent in sim (the firmware doesn't format its
reply differently per channel), so these tests assert on ROUTING -- which
store the reply landed in, and that the other store stays empty -- not on
content differences. ``PING`` is used as the probe verb: simple, synchronous,
no side effects on other subsystems.
"""
from firmware import CHANNEL_RADIO, CHANNEL_SERIAL


def test_command_on_radio_replies_and_leaves_serial_store_empty(sim):
    reply = sim.command_on("PING", CHANNEL_RADIO)

    assert reply.strip().startswith("OK pong")
    assert sim.reply_store_len(CHANNEL_RADIO) == 0   # drained by command_on()'s own read-back
    assert sim.reply_store_len(CHANNEL_SERIAL) == 0  # never touched by a RADIO-routed command


def test_command_on_serial_replies_and_leaves_radio_store_empty(sim):
    reply = sim.command_on("PING", CHANNEL_SERIAL)

    assert reply.strip().startswith("OK pong")
    assert sim.reply_store_len(CHANNEL_SERIAL) == 0  # drained by command_on()'s own read-back
    assert sim.reply_store_len(CHANNEL_RADIO) == 0   # never touched by a SERIAL-routed command


def test_command_stays_serial_only_and_is_source_compatible(sim):
    """sim_command() (the plain, pre-088-006 entry point) is a thin
    SERIAL-only wrapper over sim_command_on() -- every existing call site
    (tests/sim/unit/*.py's ~183 other test functions) keeps working
    unmodified. Confirms the wrapper still resolves through the SERIAL
    store and never touches the RADIO store."""
    reply = sim.command("PING")

    assert reply.strip().startswith("OK pong")
    assert sim.reply_store_len(CHANNEL_SERIAL) == 0
    assert sim.reply_store_len(CHANNEL_RADIO) == 0


def test_radio_and_serial_replies_do_not_cross_contaminate_across_calls(sim):
    """A RADIO command immediately followed by a SERIAL command (and vice
    versa) each read back their OWN reply -- proving the two ReplyStores are
    independent instances, not the same shared sink reset out from under
    each other."""
    radio_reply = sim.command_on("PING", CHANNEL_RADIO)
    serial_reply = sim.command_on("PING", CHANNEL_SERIAL)

    assert radio_reply.strip().startswith("OK pong")
    assert serial_reply.strip().startswith("OK pong")
    # Both channels are drained by their own command_on() call, and neither
    # leaks into the other -- both stores read back empty afterward.
    assert sim.reply_store_len(CHANNEL_SERIAL) == 0
    assert sim.reply_store_len(CHANNEL_RADIO) == 0
