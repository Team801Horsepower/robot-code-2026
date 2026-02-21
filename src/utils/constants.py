from dataclasses import dataclass


@dataclass(frozen=True)
class SwerveModuleCanIds:
    drive_motor_id: int
    steer_motor_id: int


class DriveConstants:
    # Update these IDs to your real robot CAN layout.
    FRONT_LEFT = SwerveModuleCanIds(drive_motor_id=1, steer_motor_id=11)
    FRONT_RIGHT = SwerveModuleCanIds(drive_motor_id=2, steer_motor_id=12)
    BACK_LEFT = SwerveModuleCanIds(drive_motor_id=3, steer_motor_id=13)
    BACK_RIGHT = SwerveModuleCanIds(drive_motor_id=4, steer_motor_id=14)

    MODULES = (
        FRONT_LEFT,
        FRONT_RIGHT,
        BACK_LEFT,
        BACK_RIGHT,
    )

    CANBUS_NAME = "rio"

    TRANSLATION_DEADBAND = 0.08
    ROTATION_DEADBAND = 0.08

    # Percent-output scaling for bring-up before closed-loop tuning.
    MAX_TRANSLATION_OUTPUT = 0.5
    MAX_ROTATION_OUTPUT = 0.4


class OperatorConstants:
    DRIVER_XBOX_PORT = 0


class QuestNavConstants:
    INPUT_TABLE = "questnav"
    OUTPUT_TABLE = "AdvantageScope/QuestNav"

    STALE_TIMEOUT_S = 0.25

    POSITION_LEN = 3
    EULER_LEN = 3
    QUAT_LEN = 4
