"""Isolated TCP -> process -> UDP integration; ephemeral localhost ports only."""
import pathlib
import socket
import struct
import subprocess
import sys
import threading
import time
import unittest

from pymavlink.dialects.v20 import common as d
from test_mavlink import flight_packet


class LoopbackTest(unittest.TestCase):
    def test_flight_stream_qgc_reply_and_stale_timeout(self):
        self.check_stream(False)

    def test_flow_stream_and_schema_parameter(self):
        self.check_stream(True)

    def check_stream(self, with_flow):
        with socket.socket() as server, socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as qgc:
            server.bind(('127.0.0.1', 0))
            server.listen(1)
            server.settimeout(5)
            qgc.bind(('127.0.0.1', 0))
            qgc.settimeout(0.1)
            stop = threading.Event()
            errors = []

            def producer():
                try:
                    with server.accept()[0] as connection:
                        seq = 0
                        start = time.monotonic()
                        frozen = None
                        while not stop.is_set():
                            # Fresh for 1.2 s, then repeated stale packets for >2 s.
                            if time.monotonic() - start < 1.2:
                                seq += 1
                                packet = flight_packet(seq=seq, target_s=seq * 0.02,
                                                       schema=3 if with_flow else 2)
                                if with_flow:
                                    packet += (1.5, -2, 0.9, 1, 0.033, 0.01, 35, 32)
                                frozen = struct.pack(f'<{len(packet)}d', *packet)
                            connection.sendall(frozen[:13])
                            connection.sendall(frozen[13:])
                            stop.wait(0.02)
                except (BrokenPipeError, ConnectionResetError, ConnectionAbortedError):
                    pass
                except Exception as exc:
                    errors.append(exc)

            thread = threading.Thread(target=producer, daemon=True)
            thread.start()
            command = [sys.executable, str(pathlib.Path(__file__).with_name('qdrone_to_qgc.py')),
                       '--source', 'flight', '--drone-ip', '127.0.0.1',
                       '--tcp-port', str(server.getsockname()[1]),
                       '--qgc-port', str(qgc.getsockname()[1])]
            process = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
            decoder = d.MAVLink(None)
            request = d.MAVLink_param_request_list_message(1, 1).pack(d.MAVLink(None))
            got, heartbeat_times, initial_params = set(), [], set()
            requested = False
            schema_param, flow_valid = None, None
            start = time.monotonic()
            try:
                while time.monotonic() - start < 4:
                    try:
                        data, peer = qgc.recvfrom(65535)
                    except socket.timeout:
                        continue
                    for message in decoder.parse_buffer(data) or []:
                        got.add(message.get_type())
                        if message.get_type() == 'HEARTBEAT':
                            heartbeat_times.append(time.monotonic() - start)
                        if message.get_type() == 'PARAM_VALUE':
                            if message.param_id == 'BRG_SCHEMA':
                                schema_param = message.param_value
                            initial_params.add(message.param_index)
                            # First collect the unsolicited complete list, then
                            # verify the normal QGC request/reply path still works.
                            if len(initial_params) == 6 and not requested:
                                qgc.sendto(request, peer)
                                requested = True
                        if message.get_type() == 'NAMED_VALUE_FLOAT' and message.name == 'flow_valid':
                            flow_valid = message.value
                self.assertTrue({'ATTITUDE', 'HIGHRES_IMU', 'BATTERY_STATUS', 'HEARTBEAT',
                                 'ACTUATOR_OUTPUT_STATUS', 'NAMED_VALUE_FLOAT', 'PARAM_VALUE'} <= got, got)
                self.assertLess(max(heartbeat_times), 2.1, heartbeat_times)
                self.assertFalse(errors, errors)
                self.assertEqual(initial_params, set(range(6)))
                self.assertEqual(schema_param, 3 if with_flow else 2)
                if with_flow:
                    self.assertEqual(flow_valid, 1)
            finally:
                stop.set()
                process.terminate()
                stdout, stderr = process.communicate(timeout=5)
                thread.join(timeout=2)
            self.assertIn('No valid telemetry for 2 seconds', stdout)
            self.assertIn('QGC PARAM_REQUEST_LIST -> 6', stdout)
            self.assertEqual(stderr, '')


if __name__ == '__main__':
    unittest.main()
