from __future__ import annotations

import importlib
from typing import TYPE_CHECKING, Callable

from commands2 import Command, InstantCommand
from commands2 import CommandScheduler
import wpilib

from utils.constants import DriveConstants

if TYPE_CHECKING:
    from phoenix6.controls import DutyCycleOut as DutyCycleOutType
    from phoenix6.hardware import TalonFX as TalonFXType

_PHOENIX_IMPORT_ERROR: ModuleNotFoundError | None
DutyCycleOutImpl: type[DutyCycleOutType] | None
TalonFXImpl: type[TalonFXType] | None

try:
    from phoenix6.controls import DutyCycleOut as _DutyCycleOut
    from phoenix6.hardware import TalonFX as _TalonFX
except ModuleNotFoundError as exc:  # pragma: no cover - depends on local install
    DutyCycleOutImpl = None
    TalonFXImpl = None
    _PHOENIX_IMPORT_ERROR = exc
else:
    DutyCycleOutImpl = _DutyCycleOut
    TalonFXImpl = _TalonFX
    _PHOENIX_IMPORT_ERROR = None


class Drive:
    def __init__(self, scheduler: CommandScheduler | None = None):
        self.scheduler = scheduler
        self._enabled = True
        self._disable_reason: str | None = None
        self._disable_reported = False
        self.tuner_drivetrain: object | None = None
        self._autonomous_factory: Callable[..., Command] | None = (
            self._try_create_tuner_auto_factory()
        )
        self.drive_motors: list[TalonFXType] = []
        self.steer_motors: list[TalonFXType] = []

        if TalonFXImpl is None or DutyCycleOutImpl is None:
            self._disable(
                "Phoenix 6 is not available; running with swerve drive disabled."
            )
            return

        self.tuner_drivetrain = self._try_create_tuner_drivetrain()
        self._init_hardware()

    def _disable(self, reason: str) -> None:
        self._enabled = False
        self._disable_reason = reason
        if not self._disable_reported:
            self._disable_reported = True
            wpilib.reportWarning(reason, False)

    def _init_hardware(self) -> None:
        if TalonFXImpl is None:
            self._disable("Phoenix 6 TalonFX class unavailable; swerve drive disabled.")
            return

        for module in DriveConstants.MODULES:
            try:
                self.drive_motors.append(
                    TalonFXImpl(module.drive_motor_id, DriveConstants.CANBUS_NAME)
                )
                self.steer_motors.append(
                    TalonFXImpl(module.steer_motor_id, DriveConstants.CANBUS_NAME)
                )
            except Exception as exc:
                self.drive_motors.clear()
                self.steer_motors.clear()
                self._disable(
                    f"Failed to initialize swerve hardware on CAN bus "
                    f"'{DriveConstants.CANBUS_NAME}': {exc}"
                )
                return

    def _try_create_tuner_drivetrain(self) -> object | None:
        # Tuner-generated projects often place this class in one of these modules.
        candidates = (
            ("generated.command_swerve_drivetrain", "CommandSwerveDrivetrain"),
            ("subsystems.command_swerve_drivetrain", "CommandSwerveDrivetrain"),
            ("command_swerve_drivetrain", "CommandSwerveDrivetrain"),
        )
        for module_name, class_name in candidates:
            try:
                module = importlib.import_module(module_name)
            except ModuleNotFoundError:
                continue

            drivetrain_type = getattr(module, class_name, None)
            if drivetrain_type is None:
                continue

            try:
                return drivetrain_type()
            except Exception as exc:  # pragma: no cover - depends on generated code
                wpilib.reportWarning(
                    f"Found {module_name}.{class_name} but failed to construct it: {exc}",
                    False,
                )
                return None
        return None

    def _try_create_tuner_auto_factory(self) -> Callable[..., Command] | None:
        candidates = (
            ("subsystems.tuner_autonomous", "build_autonomous_command"),
            ("generated.tuner_autonomous", "build_autonomous_command"),
            ("tuner_autonomous", "build_autonomous_command"),
        )
        for module_name, fn_name in candidates:
            try:
                module = importlib.import_module(module_name)
            except ModuleNotFoundError:
                continue

            factory = getattr(module, fn_name, None)
            if callable(factory):
                return factory
        return None

    def _apply_deadband(self, value: float, deadband: float) -> float:
        if abs(value) < deadband:
            return 0.0
        return value

    def _clamp(self, value: float) -> float:
        return max(-1.0, min(1.0, value))

    def drive(
        self,
        x_displacement: float,
        y_displacement: float,
        rotation: float,
        field_oriented: bool = False,
    ) -> None:
        if not self._enabled:
            return

        if self.tuner_drivetrain is not None:
            # If your Tuner drivetrain exposes a drive(...) function, prefer it.
            tuner_drive = getattr(self.tuner_drivetrain, "drive", None)
            if callable(tuner_drive):
                try:
                    tuner_drive(
                        x_displacement, y_displacement, rotation, field_oriented
                    )
                    return
                except TypeError:
                    pass
                except Exception as exc:
                    self._disable(f"Tuner drivetrain drive call failed: {exc}")
                    return

        x = self._apply_deadband(x_displacement, DriveConstants.TRANSLATION_DEADBAND)
        y = self._apply_deadband(y_displacement, DriveConstants.TRANSLATION_DEADBAND)
        omega = self._apply_deadband(rotation, DriveConstants.ROTATION_DEADBAND)

        x *= DriveConstants.MAX_TRANSLATION_OUTPUT
        y *= DriveConstants.MAX_TRANSLATION_OUTPUT
        omega *= DriveConstants.MAX_ROTATION_OUTPUT

        # Open-loop bring-up mix while waiting on full swerve request wiring.
        drive_outputs = (
            self._clamp(x + y + omega),  # front left
            self._clamp(x - y - omega),  # front right
            self._clamp(x - y + omega),  # back left
            self._clamp(x + y - omega),  # back right
        )
        if DutyCycleOutImpl is None:
            self._disable("Phoenix 6 DutyCycleOut unavailable during drive call.")
            return

        for motor, output in zip(self.drive_motors, drive_outputs):
            try:
                motor.set_control(DutyCycleOutImpl(output))
            except Exception as exc:
                self._disable(f"Swerve drive output failed: {exc}")
                return

        # Steer motors are initialized and controlled directly for basic validation.
        steer_output = self._clamp(omega)
        for motor in self.steer_motors:
            try:
                motor.set_control(DutyCycleOutImpl(steer_output))
            except Exception as exc:
                self._disable(f"Swerve steer output failed: {exc}")
                return

    def stop(self) -> None:
        if DutyCycleOutImpl is None:
            return
        for motor in self.drive_motors:
            try:
                motor.set_control(DutyCycleOutImpl(0.0))
            except Exception as exc:
                self._disable(f"Swerve stop failed on drive motor: {exc}")
                return
        for motor in self.steer_motors:
            try:
                motor.set_control(DutyCycleOutImpl(0.0))
            except Exception as exc:
                self._disable(f"Swerve stop failed on steer motor: {exc}")
                return

    def get_autonomous_command(self) -> Command:
        if not self._enabled:
            return InstantCommand(lambda: None)

        if self.tuner_drivetrain is not None:
            getter = getattr(self.tuner_drivetrain, "get_autonomous_command", None)
            if callable(getter):
                try:
                    command = getter()
                except Exception as exc:
                    self._disable(f"Tuner drivetrain autonomous getter failed: {exc}")
                    return InstantCommand(lambda: None)
                if command is not None and hasattr(command, "schedule"):
                    return command

        if self._autonomous_factory is not None:
            try:
                command = self._autonomous_factory()
            except TypeError:
                try:
                    command = self._autonomous_factory(self)  # type: ignore[misc]
                except Exception as exc:
                    self._disable(f"Autonomous factory invocation failed: {exc}")
                    return InstantCommand(lambda: None)
            except Exception as exc:
                self._disable(f"Autonomous factory invocation failed: {exc}")
                return InstantCommand(lambda: None)

            if command is not None and hasattr(command, "schedule"):
                return command

        wpilib.reportWarning(
            "No Phoenix 6 Tuner autonomous command source found; using stop command.",
            False,
        )
        return InstantCommand(self.stop)
