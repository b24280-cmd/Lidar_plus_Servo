#!/usr/bin/env python3
"""
phyphox_bridge: poll a phyphox "Remote access" HTTP server and republish the
sensor streams as standard ROS 2 messages.

Target: ROS 2 Jazzy (Python 3.12).

Phyphox remote API used here:
  GET /get?<buffer>=<threshold>|<refbuffer>&...  -> JSON {"buffer": {...}, "status": {...}}
       The threshold|reference syntax returns only buffer values whose matching
       reference-buffer entry is greater than <threshold>, i.e. only NEW data
       since the last poll.
  GET /control?cmd=start|stop|clear

Buffer names below come straight from the experiment config you exported
(acc_time/accX..., gyr_time/gyrX..., attT/attW..., loc_time/locLat..., etc.).

Topics published:
  imu/data        sensor_msgs/Imu            (accel + gyro, orientation if available)
  imu/mag         sensor_msgs/MagneticField  (converted uT -> T)
  fix             sensor_msgs/NavSatFix       (GPS)
  illuminance     sensor_msgs/Illuminance     (light sensor)

NOTE on frames: phyphox axes are the phone's screen frame (x=right, y=up,
z=out-of-screen). ROS body frame (REP-103) is FLU (x=forward, y=left, z=up).
Data here is passed through unrotated under frame_id=phyphox_phone. Add a
static_transform_publisher (or rotate in code) to align with your drone body.
"""

import threading
import time as pytime

import requests

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from builtin_interfaces.msg import Time as TimeMsg

from sensor_msgs.msg import Imu, MagneticField, NavSatFix, NavSatStatus, Illuminance


class PhyphoxBridge(Node):
    def __init__(self):
        super().__init__("phyphox_bridge")

        # ----- parameters -------------------------------------------------
        self.declare_parameter("host", "10.154.225.107:8080")
        self.declare_parameter("rate_hz", 25.0)
        self.declare_parameter("timeout_s", 1.0)
        self.declare_parameter("frame_id", "phyphox_phone")
        self.declare_parameter("auto_start", True)    # send /control?cmd=start on launch
        self.declare_parameter("skip_backlog", True)  # ignore data recorded before node start
        self.declare_parameter("sync_tol_s", 0.02)    # accel<->gyro pairing tolerance

        self.host = self.get_parameter("host").value
        self.base = f"http://{self.host}"
        self.rate = float(self.get_parameter("rate_hz").value)
        self.timeout = float(self.get_parameter("timeout_s").value)
        self.frame_id = self.get_parameter("frame_id").value
        self.auto_start = bool(self.get_parameter("auto_start").value)
        self.skip_backlog = bool(self.get_parameter("skip_backlog").value)
        self.sync_tol = float(self.get_parameter("sync_tol_s").value)

        # ----- publishers -------------------------------------------------
        q = qos_profile_sensor_data
        self.pub_imu = self.create_publisher(Imu, "imu/data", q)
        self.pub_mag = self.create_publisher(MagneticField, "imu/mag", q)
        self.pub_fix = self.create_publisher(NavSatFix, "fix", q)
        self.pub_lux = self.create_publisher(Illuminance, "illuminance", q)

        # ----- state ------------------------------------------------------
        # last phyphox timestamp (seconds, relative to experiment start) per group
        self.last = {"acc": 0.0, "gyr": 0.0, "mag": 0.0,
                     "att": 0.0, "loc": 0.0, "light": 0.0}
        # maps phyphox relative time -> ROS time so inter-sample timing is preserved
        self.t_offset = None
        self.primed = False

        self.session = requests.Session()

        if self.auto_start:
            self._control("start")

        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()
        self.get_logger().info(
            f"phyphox_bridge polling {self.base} at {self.rate:.1f} Hz")

    # ====================================================================
    # phyphox HTTP helpers
    # ====================================================================
    def _control(self, cmd):
        try:
            self.session.get(f"{self.base}/control",
                             params={"cmd": cmd}, timeout=self.timeout)
            self.get_logger().info(f"sent control cmd={cmd}")
        except requests.RequestException as e:
            self.get_logger().warn(f"control '{cmd}' failed: {e}")

    def _build_specs(self):
        """One /get request fetches new samples for every group at once."""
        specs = {}

        def grp(refbuf, comps, key):
            t = self.last[key]
            specs[refbuf] = f"{t}|{refbuf}"
            for c in comps:
                specs[c] = f"{t}|{refbuf}"

        grp("acc_time", ["accX", "accY", "accZ"], "acc")
        grp("gyr_time", ["gyrX", "gyrY", "gyrZ"], "gyr")
        grp("mag_time", ["magX", "magY", "magZ"], "mag")
        grp("attT", ["attW", "attX", "attY", "attZ"], "att")
        grp("loc_time", ["locLat", "locLon", "locZ", "locV",
                         "locDir", "locAccuracy", "locZAccuracy"], "loc")
        grp("light_time", ["light"], "light")
        return specs

    def _loop(self):
        period = 1.0 / self.rate if self.rate > 0 else 0.04
        while not self._stop.is_set() and rclpy.ok():
            t0 = pytime.monotonic()
            try:
                r = self.session.get(f"{self.base}/get",
                                     params=self._build_specs(),
                                     timeout=self.timeout)
                r.raise_for_status()
                self._handle(r.json().get("buffer", {}))
            except requests.RequestException as e:
                self.get_logger().warn(f"poll failed: {e}",
                                       throttle_duration_sec=2.0)
            except ValueError as e:
                self.get_logger().warn(f"bad JSON from phyphox: {e}",
                                       throttle_duration_sec=2.0)
            slack = period - (pytime.monotonic() - t0)
            if slack > 0:
                pytime.sleep(slack)

    # ====================================================================
    # parsing helpers
    # ====================================================================
    @staticmethod
    def _arr(data, name):
        b = data.get(name)
        return b.get("buffer", []) if b else []

    def _samples(self, data, tbuf, comps):
        """Return [(t, [comp0, comp1, ...]), ...] sorted ascending by t,
        dropping any rows containing nulls."""
        t = self._arr(data, tbuf)
        cols = [self._arr(data, c) for c in comps]
        if not t:
            return []
        n = min([len(t)] + [len(c) for c in cols])
        rows = []
        for i in range(n):
            tv = t[i]
            if tv is None:
                continue
            vals = [cols[j][i] for j in range(len(cols))]
            if any(v is None for v in vals):
                continue
            rows.append((float(tv), [float(v) for v in vals]))
        rows.sort(key=lambda x: x[0])
        return rows

    @staticmethod
    def _nearest(rows, t, tol):
        best, bd = None, tol
        for rt, v in rows:
            d = abs(rt - t)
            if d <= bd:
                bd, best = d, v
        return best

    def _stamp(self, ptime):
        """phyphox relative time -> ROS Time message."""
        t = self.t_offset + ptime
        m = TimeMsg()
        m.sec = int(t)
        m.nanosec = int(round((t - int(t)) * 1e9))
        return m

    # ====================================================================
    # main dispatch
    # ====================================================================
    def _handle(self, data):
        acc = self._samples(data, "acc_time", ["accX", "accY", "accZ"])
        gyr = self._samples(data, "gyr_time", ["gyrX", "gyrY", "gyrZ"])
        att = self._samples(data, "attT", ["attW", "attX", "attY", "attZ"])
        mag = self._samples(data, "mag_time", ["magX", "magY", "magZ"])
        lux = self._samples(data, "light_time", ["light"])
        loc = self._samples(data, "loc_time",
                            ["locLat", "locLon", "locZ", "locV",
                             "locDir", "locAccuracy", "locZAccuracy"])

        groups = {"acc": acc, "gyr": gyr, "att": att,
                  "mag": mag, "light": lux, "loc": loc}

        # advance the "last seen" watermark for every group
        for key, rows in groups.items():
            if rows:
                self.last[key] = max(self.last[key], rows[-1][0])

        # establish the phyphox->ROS time offset on the first batch that has data
        if self.t_offset is None:
            maxt = max((rows[-1][0] for rows in groups.values() if rows),
                       default=None)
            if maxt is None:
                return  # nothing recorded yet; wait
            ros_now = self.get_clock().now().nanoseconds * 1e-9
            self.t_offset = ros_now - maxt
            self.primed = True
            if self.skip_backlog:
                # Per-group watermark, so slow sensors (GPS) aren't starved by
                # the fast IMU clock. Each group skips only its OWN backlog.
                for key, rows in groups.items():
                    if rows:
                        self.last[key] = rows[-1][0]
                return  # don't publish the pre-start backlog

        if not self.primed:
            return

        self._pub_imu(acc, gyr, att)
        self._pub_mag(mag)
        self._pub_lux(lux)
        self._pub_loc(loc)

    def _pub_imu(self, acc, gyr, att):
        # One Imu per accelerometer sample; nearest gyro/orientation paired in.
        for t, a in acc:
            msg = Imu()
            msg.header.stamp = self._stamp(t)
            msg.header.frame_id = self.frame_id
            msg.linear_acceleration.x, msg.linear_acceleration.y, \
                msg.linear_acceleration.z = a

            g = self._nearest(gyr, t, self.sync_tol)
            if g:
                msg.angular_velocity.x, msg.angular_velocity.y, \
                    msg.angular_velocity.z = g

            q = self._nearest(att, t, self.sync_tol)
            if q:
                # phyphox order: [w, x, y, z]
                msg.orientation.w, msg.orientation.x, \
                    msg.orientation.y, msg.orientation.z = q
            else:
                msg.orientation_covariance[0] = -1.0  # orientation unknown

            self.pub_imu.publish(msg)

    def _pub_mag(self, mag):
        for t, m in mag:
            msg = MagneticField()
            msg.header.stamp = self._stamp(t)
            msg.header.frame_id = self.frame_id
            # phyphox reports microtesla; ROS wants tesla
            msg.magnetic_field.x = m[0] * 1e-6
            msg.magnetic_field.y = m[1] * 1e-6
            msg.magnetic_field.z = m[2] * 1e-6
            self.pub_mag.publish(msg)

    def _pub_lux(self, lux):
        for t, v in lux:
            msg = Illuminance()
            msg.header.stamp = self._stamp(t)
            msg.header.frame_id = self.frame_id
            msg.illuminance = v[0]
            self.pub_lux.publish(msg)

    def _pub_loc(self, loc):
        for t, v in loc:
            lat, lon, alt, vel, direction, hacc, vacc = v
            msg = NavSatFix()
            msg.header.stamp = self._stamp(t)
            msg.header.frame_id = self.frame_id
            msg.status.status = NavSatStatus.STATUS_FIX
            msg.status.service = NavSatStatus.SERVICE_GPS
            msg.latitude = lat
            msg.longitude = lon
            msg.altitude = alt
            msg.position_covariance_type = \
                NavSatFix.COVARIANCE_TYPE_APPROXIMATED
            msg.position_covariance[0] = hacc * hacc
            msg.position_covariance[4] = hacc * hacc
            msg.position_covariance[8] = vacc * vacc
            self.pub_fix.publish(msg)

    def destroy_node(self):
        self._stop.set()
        super().destroy_node()


def main():
    rclpy.init()
    node = PhyphoxBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
