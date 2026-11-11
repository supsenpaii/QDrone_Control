"""MAVLink mapping for all capture fields and the passive flight schema.

No flight-control requests or parameter writes are forwarded to hardware.
"""
import math


CAPTURE_NAMES = (
    'magic schema seq target_s analog0 analog1 analog2 analog3 battery_v '
    'gyro_x gyro_y gyro_z accel_x accel_y accel_z mag_x_T mag_y_T mag_z_T '
    'press_Pa temp_C flow_du flow_dv flow_qual flow_valid flow_dt flow_age '
    'textured inliers frame_seq frame_s'
).split()
FLIGHT_NAMES = (
    'magic schema seq target_s battery_v low_batt sens_fail comm_loss arm mode stop_cmd '
    'thrust_N cmd_roll cmd_pitch cmd_yawrat '
    'roll pitch yaw rate_x rate_y rate_z angacc_x angacc_y angacc_z '
    'gyro_x gyro_y gyro_z accel_x accel_y accel_z '
    'motor0 motor1 motor2 motor3 duty0 duty1 duty2 duty3 '
    'force_N torque_x torque_y torque_z lim_trq_x lim_trq_y lim_trq_z '
    'cmd_rate_x cmd_rate_y cmd_rate_z ext_trq_x ext_trq_y ext_trq_z '
    'in_roll in_pitch in_yawrate mag_x_T mag_y_T mag_z_T press_Pa temp_C'
).split()
assert len(CAPTURE_NAMES) == 30 and len(FLIGHT_NAMES) == 59
FLOW_NAMES = 'flow_du flow_dv flow_qual flow_valid flow_dt flow_age textured inliers'.split()
FLIGHT_FLOW_NAMES = FLIGHT_NAMES + FLOW_NAMES
assert len(FLIGHT_FLOW_NAMES) == 67
assert all(len(n) <= 10 for n in CAPTURE_NAMES + FLIGHT_FLOW_NAMES)


def values(packet):
    names = {1: CAPTURE_NAMES, 2: FLIGHT_NAMES, 3: FLIGHT_FLOW_NAMES}[packet[1]]
    return dict(zip(names, packet))


def valid(x):
    return math.isfinite(x) and abs(x) < 1e30


class Publisher:
    def __init__(self, mav, dialect):
        self.mav, self.d = mav, dialect
        self.previous_status = None

    def fast(self, packet):
        v = values(packet)
        # Quanser body FLU -> MAVLink FRD: diag(1,-1,-1).
        # https://docs.quanser.com/quarc/documentation/quarc_using_devices_standard_axes.html
        # Magnetic T -> gauss; pressure Pa -> hPa. Named values retain native axes.
        fields = [('accel_x', 1), ('accel_y', -1), ('accel_z', -1),
                  ('gyro_x', 1), ('gyro_y', -1), ('gyro_z', -1),
                  ('mag_x_T', 1e4), ('mag_y_T', -1e4), ('mag_z_T', -1e4),
                  ('press_Pa', 0.01), (None, 1), (None, 1), ('temp_C', 1)]
        data, mask = [], 0
        for i, (key, scale) in enumerate(fields):
            value = v.get(key, float('nan'))
            if valid(value):
                data.append(value * scale)
                mask |= 1 << i
            else:
                data.append(float('nan'))
        self.mav.highres_imu_send(int(v['target_s'] * 1e6), *data, mask)
        if packet[1] in (2, 3):
            attitude = [v[k] for k in ('roll', 'pitch', 'yaw', 'rate_x', 'rate_y', 'rate_z')]
            if all(valid(x) for x in attitude):
                # Change both the FLU body and model's up-positive reference.
                # Yaw remains relative to the model's initial heading, not true north.
                attitude = [x * s for x, s in zip(attitude, (1, -1, -1, 1, -1, -1))]
                self.mav.attitude_send(round(v['target_s'] * 1000) & 0xFFFFFFFF, *attitude)
            motors = [v[f'motor{i}'] for i in range(4)]
            if all(valid(x) for x in motors):
                self.mav.actuator_output_status_send(int(v['target_s'] * 1e6), 0xF,
                                                     motors + [0.0] * 28)

    def slow(self, packet):
        v, d = values(packet), self.d
        flight = packet[1] in (2, 3)
        # Flight schema arm is read from the plant input that gates motor commands.
        armed = flight and v['arm'] > 0.5
        fault = flight and any(v[k] > 0.5 for k in ('sens_fail', 'low_batt', 'comm_loss'))
        state = d.MAV_STATE_CRITICAL if fault else (d.MAV_STATE_ACTIVE if armed else d.MAV_STATE_STANDBY)
        base_mode = d.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED if flight else 0
        if armed:
            base_mode |= d.MAV_MODE_FLAG_SAFETY_ARMED
        # Keep the QUARC mode number; never claim to be PX4/ArduPilot.
        self.mav.heartbeat_send(d.MAV_TYPE_QUADROTOR, d.MAV_AUTOPILOT_GENERIC,
                                base_mode, int(v['mode']) if flight else 0, state)
        volts = v['battery_v']
        mv = round(volts * 1000) if valid(volts) and 0 < volts < 65.534 else 65535
        present = d.MAV_SYS_STATUS_SENSOR_3D_GYRO | d.MAV_SYS_STATUS_SENSOR_3D_ACCEL
        # Global model sensor failure cannot diagnose individual axes.
        health = 0 if flight and v['sens_fail'] > 0.5 else present
        self.mav.sys_status_send(present, present, health, 0, mv, -1, -1, 0, 0, 0, 0, 0, 0)
        if mv != 65535:
            self.mav.battery_status_send(0, d.MAV_BATTERY_FUNCTION_ALL, d.MAV_BATTERY_TYPE_UNKNOWN,
                                         32767, [mv] + [65535] * 9, -1, -1, -1, -1)
        status = (flight, armed, fault, int(v['mode']) if flight else 0)
        if status != self.previous_status:
            text = (f'QUARC monitor: arm={int(armed)} mode={status[3]} fault={int(fault)}'
                    if flight else 'QUARC raw capture: arm/mode unavailable')
            self.mav.statustext_send(d.MAV_SEVERITY_INFO, text.encode('ascii')[:50])
            self.previous_status = status

    def named(self, packet):
        # Every available source field is visible in Inspector/telemetry logs,
        # including controller commands without equivalent standard MAVLink units.
        ms = round(packet[3] * 1000) & 0xFFFFFFFF
        for name, value in list(values(packet).items())[2:]:
            if valid(value):
                self.mav.named_value_float_send(ms, name.encode('ascii'), value)


class QgcService:
    """Discovery and read-only bridge settings, not QUARC/PID parameters."""

    def __init__(self, mav, dialect, args):
        self.mav, self.d, self.args = mav, dialect, args
        self.parser = dialect.MAVLink(None)
        self.parser.robust_parsing = True
        self.parameter_requests = 0
        self.parameter_values_sent = 0
        self.params = {'BRG_SCHEMA': 2 if args.source == 'flight' else 1,
                       'BRG_SYSID': args.system_id, 'BRG_TCP_PORT': args.tcp_port,
                       'BRG_READONLY': 1, 'BRG_IMU_HZ': 20, 'BRG_NAMED_HZ': 2}

    def version(self):
        caps = self.d.MAV_PROTOCOL_CAPABILITY_MAVLINK2 | self.d.MAV_PROTOCOL_CAPABILITY_PARAM_ENCODE_C_CAST
        self.mav.autopilot_version_send(caps, 0, 0, 0, 0, [0]*8, [0]*8, [0]*8, 0, 0, 0)

    def param(self, index):
        if 0 <= index < len(self.params):
            name, value = list(self.params.items())[index]
            self.mav.param_value_send(name.encode('ascii'), float(value), self.d.MAV_PARAM_TYPE_REAL32,
                                       len(self.params), index)
            self.parameter_values_sent += 1

    def announce(self):
        # A generic vehicle can remain waiting if initial discovery/list requests
        # were missed or timed out. Indexed PARAM_VALUE broadcasts also let QGC
        # complete discovery on late connection; normal request/retry still works.
        self.version()
        for index in range(len(self.params)):
            self.param(index)

    def handle(self, message):
        # Broadcast targets are permitted; ignore messages intended for another vehicle.
        if getattr(message, 'target_system', self.args.system_id) not in (0, self.args.system_id):
            return
        if getattr(message, 'target_component', 1) not in (0, 1):
            return
        kind, d = message.get_type(), self.d
        if kind == 'PARAM_REQUEST_LIST':
            self.parameter_requests += 1
            print(f'QGC PARAM_REQUEST_LIST -> {len(self.params)} read-only bridge parameters', flush=True)
            for i in range(len(self.params)):
                self.param(i)
        elif kind in ('PARAM_REQUEST_READ', 'PARAM_SET'):
            self.parameter_requests += 1
            name = message.param_id
            if isinstance(name, bytes):
                name = name.decode('ascii', errors='replace')
            name = name.rstrip('\x00')
            index = getattr(message, 'param_index', -1)
            if index < 0:
                index = list(self.params).index(name) if name in self.params else -1
            self.param(index)
            if kind == 'PARAM_SET':
                self.mav.statustext_send(d.MAV_SEVERITY_WARNING, b'Bridge settings are read-only; use Simulink for PID')
        elif kind == 'COMMAND_LONG':
            ok = False
            if message.command == d.MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES:
                self.version()
                ok = True
            elif message.command == d.MAV_CMD_REQUEST_MESSAGE and int(message.param1) == 148:
                self.version()
                ok = True
            self.mav.command_ack_send(message.command,
                d.MAV_RESULT_ACCEPTED if ok else d.MAV_RESULT_UNSUPPORTED,
                target_system=message.get_srcSystem(), target_component=message.get_srcComponent())
        elif kind == 'COMMAND_INT':
            self.mav.command_ack_send(message.command, d.MAV_RESULT_UNSUPPORTED,
                target_system=message.get_srcSystem(), target_component=message.get_srcComponent())
        elif kind == 'MISSION_REQUEST_LIST':
            self.mav.mission_count_send(message.get_srcSystem(), message.get_srcComponent(), 0,
                                        getattr(message, 'mission_type', 0))
        elif kind in ('MISSION_COUNT', 'MISSION_CLEAR_ALL'):
            self.mav.mission_ack_send(message.get_srcSystem(), message.get_srcComponent(),
                d.MAV_MISSION_DENIED, getattr(message, 'mission_type', 0))
