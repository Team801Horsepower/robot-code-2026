# from rev import CANSparkFlex  # Import vendor library


# class MotorExample:
#     def __init__(
#         self, motor_can_id: int
#     ):  # The class constructor takes one parameter: the CAN ID of the motor
#         self.motor = CANSparkFlex(
#             motor_can_id, CANSparkFlex.MotorType.kBrushless
#         )  # Create the motor object

#     def move_motor(
#         self, power: float
#     ):  # This method can be used to set the power of the motor
#         self.motor.set(power)  # Call into the vendor library to run the motor
