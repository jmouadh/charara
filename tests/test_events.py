"""Board event log & response-integrity tests.

Covers the two review findings on the RPi side:
* a response whose echoed ``seq`` is not the request's (the board had not
  executed it yet) must be retried, never accepted;
* a burst of more than five board events between two polls must not lose
  events: the drain cursor stops at the newest READ entry.

File:    tests/test_events.py
Author:  Mouadh Jeridi <jeridimouadh@gmail.com>
Version: 3.0.0
Date:    2026-09-28
"""

from __future__ import annotations

import unittest

from software.console.runtime.state import advance_event_cursor
from software.robot import protocol as p
from software.robot.client import RobotClient
from software.robot.link import LinkError, MockLink


def make_response(seq: int, status: int, payload: bytes = b"") -> bytes:
    """Build a valid 32-byte response frame (CRC correct)."""
    body = bytes([p.SOF_RESPONSE, seq & 0xFF, status & 0xFF, len(payload)]) + payload.ljust(
        p.RSP_PAYLOAD_SIZE, b"\x00"
    )
    return body + bytes([p.crc8(body)])


class ScriptedLink:
    """Byte-level link replaying a fixed list of response frames."""

    def __init__(self, responses: list[bytes]) -> None:
        """Init."""
        self._responses = list(responses)
        self.calls = 0

    def transact(self, request: bytes) -> bytes:
        """Transact."""
        self.calls += 1
        return self._responses.pop(0)

    def close(self) -> None:
        """Close."""
        pass


class StaleSeqTests(unittest.IsolatedAsyncioTestCase):
    async def test_stale_seq_is_retried_with_same_seq(self) -> None:
        """First read serves the previous command's frame: retry, then OK."""
        stale = make_response(seq=41, status=p.ST_OK)  # someone else's seq
        fresh = make_response(seq=42, status=p.ST_OK, payload=p.put_u32(0xCAFEBABE))
        link = ScriptedLink([stale, fresh])
        client = RobotClient(link, retries=3)
        client._seq = 41  # the next request carries seq 42

        echo = await client.ping(0xCAFEBABE)
        self.assertEqual(echo, 0xCAFEBABE)
        self.assertEqual(link.calls, 2)

    async def test_persistent_stale_seq_fails_as_link_error(self) -> None:
        """Persistent stale seq fails as link error."""
        link = ScriptedLink([make_response(0, p.ST_OK), make_response(0, p.ST_OK)])
        client = RobotClient(link, retries=2)

        with self.assertRaises(LinkError):
            await client.transact(p.CMD_PING, p.put_u32(1))
        self.assertEqual(link.calls, 2)

    async def test_matching_seq_is_accepted_first_try(self) -> None:
        """Matching seq is accepted first try."""
        link = ScriptedLink([make_response(7, p.ST_OK)])
        client = RobotClient(link, retries=3)
        client._seq = 6  # next seq is 7

        status, _ = await client.transact(p.CMD_PING, p.put_u32(2))
        self.assertEqual(status, p.ST_OK)
        self.assertEqual(link.calls, 1)


class EventCursorTests(unittest.TestCase):
    def _event(self, index: int) -> dict:
        """Event."""
        return {"index": index, "code": p.EVENT_BUSY_REJECT, "aux": 0x10}

    def test_burst_stops_at_newest_read_entry(self) -> None:
        # 7 events pending, the board answers the first 5: never skip 6-7.
        """Burst stops at newest read entry."""
        events = [self._event(i) for i in range(1, 6)]
        cursor = advance_event_cursor(0, events, latest=7)
        self.assertEqual(cursor, 5)
        # next poll continues exactly where the last one stopped
        cursor = advance_event_cursor(cursor, [self._event(6), self._event(7)], latest=7)
        self.assertEqual(cursor, 7)

    def test_empty_read_jumps_to_latest(self) -> None:
        # everything newer was overwritten in the board ring: resync
        """Empty read jumps to latest."""
        self.assertEqual(advance_event_cursor(3, [], latest=9), 9)
        # nothing new at all: cursor unchanged
        self.assertEqual(advance_event_cursor(9, [], latest=9), 9)

    def test_normal_read_advances_to_newest(self) -> None:
        """Normal read advances to newest."""
        events = [self._event(3), self._event(4)]
        self.assertEqual(advance_event_cursor(2, events, latest=4), 4)

    def test_mock_link_drain_loses_nothing(self) -> None:
        """End to end on the mock: 7 recorded events are all drained."""
        link = MockLink()
        for _ in range(7):
            link._record(p.EVENT_BUSY_REJECT, 0x10)

        async def drain() -> list[int]:
            """Drain."""
            client = RobotClient(link, retries=1)
            cursor = 0
            seen: list[int] = []
            for _ in range(5):  # plenty of polls for a 7-event burst
                result = await client.events(cursor)
                seen.extend(event["index"] for event in result["events"])
                cursor = advance_event_cursor(cursor, result["events"], int(result.get("latest", 0)))
                if not result["events"]:
                    break
            return seen

        import asyncio

        seen = asyncio.run(drain())
        # 1 boot event (RESET_POWER_ON) + 7 recorded
        self.assertEqual(seen, list(range(1, 9)))


if __name__ == "__main__":
    unittest.main()
