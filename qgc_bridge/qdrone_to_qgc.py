"""Read QDrone capture or Stabilizer_QGC telemetry and publish MAVLink to QGC.

No bytes are sent to the drone. QGC commands are not forwarded.
Requires Python 3.10+ and pymavlink; see HUONG_DAN.md.
"""
import argparse
import math
import socket
import struct
import time

from telemetry import Publisher, QgcService


class TelemetryParser:
    """Capture 1 / flight 2 / flight+flow 3, little endian, no dimensions."""

    HEADER = struct.pack('<2d', 51234567, 1)
    PACKET = struct.Struct('<30d')
    SIZES = {1: 30, 2: 59, 3: 67}
    MAGIC = struct.pack('<d', 51234567)

    def __init__(self):
        self.buffer = bytearray()
        self.count = 0

    def feed(self, data):
        self.buffer.extend(data)
        latest = None
        while True:
            start = self.buffer.find(self.MAGIC)
            if start < 0:
                # Keep a possible fragmented header.
                self.buffer[:] = self.buffer[-(len(self.HEADER) - 1):]
                break
            if start:
                del self.buffer[:start]
            if len(self.buffer) < 16:
                break
            schema = struct.unpack_from('<d', self.buffer, 8)[0]
            if schema not in self.SIZES:
                del self.buffer[0]
                continue
            size = self.SIZES[schema] * 8
            if len(self.buffer) < size:
                break
            p = struct.unpack_from(f'<{self.SIZES[schema]}d', self.buffer)
            if not all(math.isfinite(v) and v >= 0 for v in p[2:4]):
                del self.buffer[0]
                continue
            del self.buffer[:size]
            latest = p
            self.count += 1
        return latest


def run(args, dialect):
    destination = (socket.gethostbyname(args.qgc_ip), args.qgc_port)
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as udp:
        udp.bind(('0.0.0.0', 0))
        udp.setblocking(False)
        class UdpWriter:
            def write(self, data):
                return udp.sendto(data, destination)

        mav = dialect.MAVLink(UdpWriter(), srcSystem=args.system_id, srcComponent=1)
        publisher = Publisher(mav, dialect)
        service = QgcService(mav, dialect, args)
        print(f'QDrone {args.drone_ip}:{args.tcp_port} -> QGC {destination}', flush=True)
        print(f'Source={args.source}. Telemetry only; QGC flight commands are rejected.', flush=True)
        while True:
            try:
                print('Connecting to telemetry...', flush=True)
                with socket.create_connection((args.drone_ip, args.tcp_port), timeout=3) as tcp:
                    tcp.settimeout(0.02)
                    parser = TelemetryParser()
                    latest = None
                    last_sample = time.monotonic()
                    heartbeat_at = sensor_at = status_at = named_at = param_at = 0.0
                    while True:
                        try:
                            data = tcp.recv(65536)
                            if not data:
                                raise ConnectionError('Drone closed telemetry stream')
                            packet = parser.feed(data)
                            if packet is not None:
                                expected = (2, 3) if args.source == 'flight' else (1,)
                                if packet[1] not in expected:
                                    raise ConnectionError(f'Expected schema {expected}, received {packet[1]}')
                                if service.params['BRG_SCHEMA'] != packet[1]:
                                    service.params['BRG_SCHEMA'] = int(packet[1])
                                    param_at = 0.0
                                if latest is None or packet[2:4] != latest[2:4]:
                                    latest = packet
                                    last_sample = time.monotonic()
                        except socket.timeout:
                            pass
                        now = time.monotonic()
                        if now - last_sample > 2:
                            raise ConnectionError('No valid telemetry for 2 seconds')
                        if latest is None:
                            continue
                        # Receive requests on the same ephemeral UDP port QGC replies to.
                        for _ in range(32):
                            try:
                                wire, peer = udp.recvfrom(65535)
                            except BlockingIOError:
                                break
                            if peer[0] != destination[0]:
                                continue
                            for message in service.parser.parse_buffer(wire) or []:
                                if message.get_type() != 'BAD_DATA':
                                    service.handle(message)
                        if now - last_sample > 0.5:
                            continue
                        if now >= sensor_at:
                            publisher.fast(latest)
                            sensor_at = now + 0.05
                        if now >= heartbeat_at:
                            publisher.slow(latest)
                            heartbeat_at = now + 1
                        if now >= param_at:
                            service.announce()
                            print(f'PARAMS advertised: {len(service.params)} BRG_* (read-only)', flush=True)
                            param_at = now + 5
                        if now >= named_at:
                            publisher.named(latest)
                            named_at = now + 0.5
                        if now >= status_at:
                            battery = latest[4] if latest[1] in (2, 3) else latest[8]
                            flow_status = (f' flow_du={latest[59]:.2f} flow_dv={latest[60]:.2f}'
                                           f' quality={latest[61]:.2f} valid={latest[62]:.0f}'
                                           f' age={latest[64]:.3f}s' if latest[1] == 3 else '')
                            print(f'LIVE packets={parser.count} target_s={latest[3]:.3f} '
                                  f'schema={latest[1]:.0f} battery={battery:.2f} V '
                                  f'param_rx={service.parameter_requests} '
                                  f'param_tx={service.parameter_values_sent}{flow_status}', flush=True)
                            status_at = now + 2
            except OSError as exc:
                # No heartbeat during reconnect: do not disguise lost drone data.
                print(f'Telemetry disconnected: {exc}. Retry in 2 s.', flush=True)
                time.sleep(2)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--drone-ip', default='192.168.2.52')
    parser.add_argument('--source', choices=('capture', 'flight'), default='capture')
    parser.add_argument('--tcp-port', type=int, default=None)
    parser.add_argument('--qgc-ip', default='127.0.0.1')
    parser.add_argument('--qgc-port', type=int, default=14550)
    parser.add_argument('--system-id', type=int, default=1, choices=range(1, 256), metavar='1..255')
    args = parser.parse_args()
    if args.tcp_port is None:
        args.tcp_port = 18920 if args.source == 'flight' else 18910
    try:
        from pymavlink.dialects.v20 import common as dialect
    except ImportError:
        parser.exit(1, 'Missing pymavlink. Run: python -m pip install -r qgc_bridge/requirements.txt\n')
    try:
        run(args, dialect)
    except KeyboardInterrupt:
        print('\nBridge stopped.')


if __name__ == '__main__':
    main()
