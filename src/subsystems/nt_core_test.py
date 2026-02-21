from __future__ import annotations

import time
from collections.abc import Sequence

import ntcore
import wpilib
from wpimath.geometry import Pose2d, Pose3d, Rotation2d, Rotation3d, Translation3d

from utils.constants import QuestNavConstants


class QuestNavNtBridge:
    def __init__(
        self,
        nt_instance: ntcore.NetworkTableInstance | None = None,
        stale_timeout_s: float = QuestNavConstants.STALE_TIMEOUT_S,
    ) -> None:
        self._nt_instance = nt_instance or ntcore.NetworkTableInstance.getDefault()
        self._stale_timeout_s = stale_timeout_s
        self._warning_interval_s = 1.0
        self._last_warning_s: dict[str, float] = {}

        ## Start the server
        self._nt_instance.startServer(QuestNavConstants.SERVER_NAME)

        input_table = self._nt_instance.getTable(QuestNavConstants.INPUT_TABLE)
        output_table = self._nt_instance.getTable(QuestNavConstants.OUTPUT_TABLE)

        self._timestamp_sub = input_table.getDoubleTopic("timestamp").subscribe(0.0)
        self._position_sub = input_table.getDoubleArrayTopic("position").subscribe([])
        self._quaternion_sub = input_table.getFloatArrayTopic("quaternion").subscribe(
            []
        )
        self._euler_sub = input_table.getFloatArrayTopic("eulerAngles").subscribe([])
        self._battery_sub = input_table.getDoubleTopic("batteryPercent").subscribe(0.0)
        self._tracking_status_sub = input_table.getIntegerTopic(
            "trackingStatus"
        ).subscribe(0)
        self._version_sub = input_table.getIntegerTopic("version").subscribe(0)

        self._pose3d_pub = output_table.getStructTopic("Pose3d", Pose3d).publish()
        self._pose2d_pub = output_table.getStructTopic("Pose2d", Pose2d).publish()
        self._connected_pub = output_table.getBooleanTopic("Connected").publish()
        self._data_age_pub = output_table.getDoubleTopic("DataAgeSec").publish()
        self._tracking_status_pub = output_table.getIntegerTopic(
            "TrackingStatus"
        ).publish()
        self._battery_pub = output_table.getDoubleTopic("BatteryPercent").publish()
        self._version_pub = output_table.getIntegerTopic("Version").publish()
        self._raw_position_pub = output_table.getDoubleArrayTopic(
            "RawPosition"
        ).publish()
        self._raw_euler_pub = output_table.getFloatArrayTopic(
            "RawEulerAngles"
        ).publish()
        self._raw_quaternion_pub = output_table.getFloatArrayTopic(
            "RawQuaternion"
        ).publish()
        self._teleop_active_pub = output_table.getBooleanTopic("TeleopActive").publish()
        self._last_timestamp_pub = output_table.getDoubleTopic(
            "LastTimestampUs"
        ).publish()

        self._connected_pub.set(False)
        self._data_age_pub.set(-1.0)
        self._teleop_active_pub.set(False)
        
        # Create + publish a Field2d for AdvantageScope
        self._field = wpilib.Field2d()
        wpilib.SmartDashboard.putData("QuestNavField", self._field)
        
        #Immediately publish a default pose to ensure the topic is populated and subscribers can get an initial value
        self._publish_default_start_pose()

    def on_teleop_enable(self) -> None:
        self._teleop_active_pub.set(True)
        self._connected_pub.set(False)
        self._data_age_pub.set(-1.0)
        
        # Reset displayed pose to a reasonable field-relative start position for 2026.
        self._publish_default_start_pose()

    def on_teleop_disable(self) -> None:
        self._teleop_active_pub.set(False)
        self._connected_pub.set(False)

    def update(self, now_s: float | None = None) -> None:
        timestamp_now_s = time.monotonic() if now_s is None else now_s
        try:
            timestamp_us = float(self._timestamp_sub.get())
            position = list(self._position_sub.get())
            quaternion = list(self._quaternion_sub.get())
            euler = list(self._euler_sub.get())
            battery_percent = float(self._battery_sub.get())
            tracking_status = int(self._tracking_status_sub.get())
            version = int(self._version_sub.get())
        except Exception as exc:
            self._warn_rate_limited(
                "questnav_read",
                f"QuestNav NT read failed: {exc}",
                timestamp_now_s,
            )
            self._connected_pub.set(False)
            self._data_age_pub.set(-1.0)
            return

        self._tracking_status_pub.set(tracking_status)
        self._battery_pub.set(battery_percent)
        self._version_pub.set(version)
        self._raw_position_pub.set(position)
        self._raw_euler_pub.set(euler)
        self._raw_quaternion_pub.set(quaternion)
        self._last_timestamp_pub.set(timestamp_us)

        if timestamp_us > 0.0:
            data_age_s = timestamp_now_s - (timestamp_us / 1_000_000.0)
        else:
            data_age_s = -1.0
        self._data_age_pub.set(data_age_s)

        position_ok = self._is_expected_length(
            "position", position, QuestNavConstants.POSITION_LEN, timestamp_now_s
        )
        euler_ok = self._is_expected_length(
            "eulerAngles", euler, QuestNavConstants.EULER_LEN, timestamp_now_s
        )
        quaternion_ok = self._is_expected_length(
            "quaternion", quaternion, QuestNavConstants.QUAT_LEN, timestamp_now_s
        )
        payload_ok = position_ok and euler_ok and quaternion_ok
        fresh = 0.0 <= data_age_s <= self._stale_timeout_s
        connected = payload_ok and fresh
        self._connected_pub.set(connected)

        if not payload_ok:
            return

        try:
            pose3d = self._to_wpilib_pose(position, euler)
            pose2d = pose3d.toPose2d()

            self._pose3d_pub.set(pose3d)
            self._pose2d_pub.set(pose2d)

            # Update Field2d visualization
            self._field.setRobotPose(pose2d)
            
        except Exception as exc:
            self._warn_rate_limited(
                "questnav_pose",
                f"QuestNav pose conversion failed: {exc}",
                timestamp_now_s,
            )
            self._connected_pub.set(False)

    def _to_wpilib_pose(
        self, position: Sequence[float], euler: Sequence[float]
    ) -> Pose3d:
        x = float(position[0])
        y = float(position[1])
        z = float(position[2])

        roll_deg = float(euler[0])
        pitch_deg = float(euler[1])
        yaw_deg = float(euler[2])

        translation = Translation3d(-z, x, y)
        rotation = Rotation3d.fromDegrees(-(pitch_deg + 90.0), -roll_deg, yaw_deg)
        return Pose3d(translation, rotation)

    def _is_expected_length(
        self,
        name: str,
        values: Sequence[float],
        expected_len: int,
        now_s: float,
    ) -> bool:
        if len(values) == expected_len:
            return True
        self._warn_rate_limited(
            f"questnav_{name}_len",
            f"QuestNav '{name}' expected len={expected_len}, got len={len(values)}",
            now_s,
        )
        return False

    def _warn_rate_limited(self, key: str, message: str, now_s: float) -> None:
        last_time_s = self._last_warning_s.get(key)
        if last_time_s is not None and (now_s - last_time_s) < self._warning_interval_s:
            return
        self._last_warning_s[key] = now_s
        wpilib.reportWarning(message, False)

    def _publish_default_start_pose(self) -> None:
        """
        Reasonable REBUILT 2026 default in WPILib/AdvantageScope 'Blue Wall' coordinates:
          x=1.00m, y=fieldWidth/2, heading=0deg (facing +X, toward red).
        """
        field_width_m = 8.07  # from 2026 manual (~8.07m)
        x_m = 1.00
        y_m = field_width_m / 2.0
        heading = Rotation2d.fromDegrees(0.0)

        pose2d = Pose2d(x_m, y_m, heading)
        pose3d = Pose3d(Translation3d(x_m, y_m, 0.0), Rotation3d(0.0, 0.0, heading.radians()))

        self._pose2d_pub.set(pose2d)
        self._pose3d_pub.set(pose3d)
        
        # Also initialize the Field2d pose
        if hasattr(self, "_field"):
            self._field.setRobotPose(pose2d)