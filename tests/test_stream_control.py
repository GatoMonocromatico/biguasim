"""Fire-and-forget control. A real socket, but no engine and no world.

``stream_control`` exists because ``set_control`` waits for an ack, and a
round trip per command caps how often a client can steer at ``1/RTT`` -- fine
for a script, fatal for a flight controller closing a loop at the world's tick
rate.

Not waiting has a cost: replies carry no correlation id, so ``_request`` takes
the next non-event message as its answer. Every test here is about that. The
failure it guards against is silent -- a client reads a stale control ack as
the answer to a spawn and believes something happened that did not.
"""
import threading
import time

import pytest
import zmq

from biguasim.client.remote import RemoteWorld
from biguasim.server import protocol as proto


class _Echo:
    """A ROUTER that answers every request, and can push unprompted events."""

    def __init__(self):
        ctx = zmq.Context.instance()
        self._socket = ctx.socket(zmq.ROUTER)
        self._socket.setsockopt(zmq.LINGER, 0)
        self.port = self._socket.bind_to_random_port("tcp://127.0.0.1")
        self.seen = []
        self._identity = None
        # ZeroMQ sockets belong to one thread. Events asked for from the test
        # thread are queued and sent by the serving thread, never sent directly.
        self._outbox = []
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._serve, daemon=True)
        self._thread.start()

    def _serve(self):
        while not self._stop.is_set():
            while self._outbox and self._identity is not None:
                self._socket.send_multipart(
                    [self._identity, proto.pack(self._outbox.pop(0))])
            if not self._socket.poll(50):
                continue
            identity, raw = self._socket.recv_multipart()
            self._identity = identity
            message = proto.unpack(raw)
            self.seen.append(message)
            op = message.get("op")
            if op == proto.OP_HELLO:
                reply = {"ok": True, "tick": 1, "input_delay": 3, "agents": {}}
            else:
                reply = {"ok": True, "scheduled": [len(self.seen)], "tick": len(self.seen)}
            self._socket.send_multipart([identity, proto.pack(reply)])

    def push_event(self, event):
        """Queue something the client never asked for, as the world does."""
        self._outbox.append(event)

    def close(self):
        self._stop.set()
        self._thread.join(timeout=2)
        self._socket.close(linger=0)


def _until(predicate, timeout=5.0):
    """Wait for something a background thread is doing.

    Sockets and threads make every assertion here a race. Spinning without
    yielding just starves the thread doing the work, so this sleeps.
    """
    deadline = time.time() + timeout
    while time.time() < deadline:
        got = predicate()
        if got:
            return got
        time.sleep(0.01)
    return None


@pytest.fixture
def world():
    server = _Echo()
    client = RemoteWorld(address="127.0.0.1", port=server.port, timeout=5.0)
    client.connect()
    yield server, client
    client._requests.close(linger=0)
    client._stream.close(linger=0)
    server.close()


def test_stream_control_does_not_wait_for_an_ack(world):
    server, client = world
    assert client.stream_control("uav0", [1.0, 2.0, 3.0, 4.0]) is None

    # The point of the exercise: it was sent, and nothing was read back.
    sent = _until(lambda: [m for m in server.seen
                           if m.get("actions", [{}])[0].get("kind") == "set_control"])
    assert sent, "the control never reached the world"
    assert client._unacked == 1, "an ack was waited for after all"


def test_a_later_request_is_not_answered_by_a_control_ack(world):
    """The bug this whole mechanism exists to prevent."""
    server, client = world
    for _ in range(5):
        client.stream_control("uav0", [0.0, 0.0, 0.0, 0.0])

    # Five acks are now in flight. Without the outstanding-ack counter this
    # spawn would be answered by the first of them.
    scheduled = client.spawn_agent("uav1", "HolybroX500")
    spawn_index = next(i for i, m in enumerate(server.seen)
                       if m.get("actions", [{}])[0].get("kind") == "spawn_agent")
    assert scheduled == spawn_index + 1


def test_events_still_arrive_while_acks_are_outstanding(world):
    """A failure notice must not be counted off as somebody's ack."""
    server, client = world
    client.stream_control("uav0", [0.0] * 4)
    # The shape service.py._report_failures actually sends.
    server.push_event({"ok": False, "event": "action_failed", "tick": 9,
                       "error": "no such agent"})

    failures = _until(client.failures)
    assert failures and failures[0]["error"] == "no such agent"


def test_the_counter_does_not_run_away(world):
    """_pump_events must count acks off too, or _request skips a real reply."""
    server, client = world
    for _ in range(3):
        client.stream_control("uav0", [0.0] * 4)

    def drained():
        client.failures()          # drives _pump_events, which eats the acks
        return client._unacked == 0
    assert _until(drained), "acks were never counted off"

    reply = client._request({"op": proto.OP_PING})
    assert reply["ok"]
