"""Real pymavlink serialization/decoding; tests never contact a drone or QGC."""
import io
import math
import struct
import unittest
from types import SimpleNamespace

from pymavlink.dialects.v20 import common as d
from qdrone_to_qgc import TelemetryParser
from telemetry import Publisher, QgcService, CAPTURE_NAMES, FLIGHT_NAMES


def flight_packet(**updates):
    data = dict.fromkeys(FLIGHT_NAMES, 0.0)
    data.update(magic=51234567, schema=2, seq=100, target_s=2,
                battery_v=12.5, arm=1, mode=1,
                roll=0.1, pitch=-0.2, yaw=0.3,
                rate_x=0.4, rate_y=0.5, rate_z=-0.6,
                gyro_x=0.4, gyro_y=0.5, gyro_z=-0.6, accel_z=9.81,
                motor0=0.2, motor1=0.3, motor2=0.4, motor3=0.5,
                mag_x_T=math.nan, mag_y_T=math.nan, mag_z_T=math.nan,
                press_Pa=math.nan, temp_C=math.nan)
    data.update(updates)
    return tuple(data[n] for n in FLIGHT_NAMES)


class MavlinkTests(unittest.TestCase):
    def setUp(self):
        self.wire = io.BytesIO()
        self.mav = d.MAVLink(self.wire, srcSystem=1, srcComponent=1)
        self.pub = Publisher(self.mav, d)
        self.service = QgcService(self.mav, d,
            SimpleNamespace(source='flight', system_id=1, tcp_port=18920))

    def messages(self):
        return d.MAVLink(None).parse_buffer(self.wire.getvalue()) or []

    def test_flight_wire_fields_and_real_arm(self):
        packet = flight_packet()
        self.pub.fast(packet)
        self.pub.slow(packet)
        self.pub.named(packet)
        messages = self.messages()
        by_type = {m.get_type(): m for m in messages}
        self.assertAlmostEqual(by_type['ATTITUDE'].pitch, 0.2, places=6)
        self.assertAlmostEqual(by_type['ATTITUDE'].yaw, -0.3, places=6)
        self.assertAlmostEqual(by_type['ATTITUDE'].yawspeed, 0.6, places=6)
        self.assertEqual(by_type['HIGHRES_IMU'].fields_updated, 0x3F)
        self.assertTrue(math.isnan(by_type['HIGHRES_IMU'].abs_pressure))
        self.assertTrue(by_type['HEARTBEAT'].base_mode & d.MAV_MODE_FLAG_SAFETY_ARMED)
        self.assertEqual(by_type['HEARTBEAT'].custom_mode, 1)
        self.assertEqual(by_type['HEARTBEAT'].autopilot, d.MAV_AUTOPILOT_GENERIC)
        self.assertEqual(by_type['BATTERY_STATUS'].voltages[0], 12500)
        self.assertAlmostEqual(by_type['ACTUATOR_OUTPUT_STATUS'].actuator[3], 0.5)
        named = {m.name: m.value for m in messages if m.get_type() == 'NAMED_VALUE_FLOAT'}
        self.assertAlmostEqual(named['battery_v'], 12.5)
        self.assertNotIn('press_Pa', named)
        self.assertIn('cmd_rate_x', named)

    def test_disarm_and_fault_from_actual_flags(self):
        self.pub.slow(flight_packet(arm=0, sens_fail=1))
        by_type = {m.get_type(): m for m in self.messages()}
        self.assertFalse(by_type['HEARTBEAT'].base_mode & d.MAV_MODE_FLAG_SAFETY_ARMED)
        self.assertEqual(by_type['HEARTBEAT'].system_status, d.MAV_STATE_CRITICAL)
        self.assertEqual(by_type['SYS_STATUS'].onboard_control_sensors_health, 0)

    def test_capture_units_and_no_invented_attitude(self):
        v = dict.fromkeys(CAPTURE_NAMES, math.nan)
        v.update(magic=51234567, schema=1, seq=1, target_s=1,
                 battery_v=12, gyro_x=0, gyro_y=0, gyro_z=0,
                 accel_x=0, accel_y=0, accel_z=9.81,
                 mag_x_T=0.00002, mag_y_T=0.00003, mag_z_T=0.00004,
                 press_Pa=101325, temp_C=30)
        p = tuple(v[n] for n in CAPTURE_NAMES)
        self.pub.fast(p)
        self.pub.slow(p)
        by_type = {m.get_type(): m for m in self.messages()}
        imu = by_type['HIGHRES_IMU']
        self.assertAlmostEqual(imu.xmag, 0.2, places=6)
        self.assertAlmostEqual(imu.ymag, -0.3, places=6)
        self.assertAlmostEqual(imu.zacc, -9.81, places=5)
        self.assertAlmostEqual(imu.abs_pressure, 1013.25)
        self.assertFalse(imu.fields_updated & (1 << 11))
        self.assertNotIn('ATTITUDE', by_type)

    def test_fragmented_flight_schema(self):
        wire = struct.pack('<59d', *flight_packet())
        for i in range(1, len(wire)):
            parser = TelemetryParser()
            self.assertIsNone(parser.feed(wire[:i]))
            self.assertEqual(parser.feed(wire[i:])[4], 12.5)

    def test_flight_flow_schema_and_pixel_units(self):
        packet = flight_packet() + (2.5, -1.25, 0.8, 1, 0.033, 0.01, 30, 28)
        wire = struct.pack('<67d', *packet[:1], 3, *packet[2:])
        parser = TelemetryParser()
        self.assertIsNone(parser.feed(wire[:471]))
        p = parser.feed(wire[471:])
        self.assertEqual(len(p), 67)
        self.pub.fast(p)
        self.pub.slow(p)
        self.pub.named(p)
        messages = self.messages()
        named = {m.name: m.value for m in messages if m.get_type() == 'NAMED_VALUE_FLOAT'}
        self.assertEqual(named['flow_du'], 2.5)
        self.assertEqual(named['flow_dv'], -1.25)
        self.assertAlmostEqual(named['flow_qual'], 0.8, places=6)
        self.assertEqual(named['flow_valid'], 1)
        self.assertTrue(any(m.get_type() == 'ATTITUDE' for m in messages))
        self.assertFalse(any(m.get_type() in ('OPTICAL_FLOW', 'OPTICAL_FLOW_RAD', 'LOCAL_POSITION_NED')
                             for m in messages))

    def test_parameters_read_only_and_wrong_vehicle_ignored(self):
        self.service.handle(d.MAVLink_param_request_list_message(2, 1))
        self.assertEqual(self.wire.getvalue(), b'')
        self.service.handle(d.MAVLink_param_request_list_message(1, 1))
        params = [m for m in self.messages() if m.get_type() == 'PARAM_VALUE']
        self.assertEqual(len(params), 6)
        self.assertTrue(all(m.param_id.startswith('BRG_') for m in params))
        self.service.handle(d.MAVLink_param_set_message(1, 1, b'BRG_SYSID', 99, d.MAV_PARAM_TYPE_REAL32))
        self.assertEqual(self.service.params['BRG_SYSID'], 1)
        params = [m for m in self.messages() if m.get_type() == 'PARAM_VALUE']
        self.assertEqual(params[-1].param_value, 1)

    def test_arm_command_is_rejected(self):
        self.service.handle(d.MAVLink_command_long_message(1, 1, d.MAV_CMD_COMPONENT_ARM_DISARM,
                                                          0, 1, 0, 0, 0, 0, 0, 0))
        ack = self.messages()[-1]
        self.assertEqual(ack.get_type(), 'COMMAND_ACK')
        self.assertEqual(ack.result, d.MAV_RESULT_UNSUPPORTED)

    def test_capability_request_and_empty_mission(self):
        self.service.handle(d.MAVLink_command_long_message(1, 1, d.MAV_CMD_REQUEST_MESSAGE,
                                                          0, 148, 0, 0, 0, 0, 0, 0))
        self.service.handle(d.MAVLink_mission_request_list_message(1, 1))
        messages = self.messages()
        self.assertEqual(messages[0].get_type(), 'AUTOPILOT_VERSION')
        self.assertEqual(messages[-1].count, 0)

    def test_announcement_recovers_missed_initial_request(self):
        self.service.announce()
        params = [m for m in self.messages() if m.get_type() == 'PARAM_VALUE']
        self.assertEqual({m.param_index for m in params}, set(range(6)))
        self.assertTrue(all(m.param_count == 6 for m in params))
        self.assertTrue(all(m.get_srcSystem() == 1 and m.get_srcComponent() == 1 for m in params))
        self.assertEqual(self.service.parameter_requests, 0)
        self.assertEqual(self.service.parameter_values_sent, 6)
        self.service.handle(d.MAVLink_param_request_read_message(1, 1, b'', 4))
        self.assertEqual(self.messages()[-1].param_index, 4)
        self.assertEqual(self.service.parameter_requests, 1)


if __name__ == '__main__':
    unittest.main()
