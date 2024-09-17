#!/usr/bin/env python3
#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#

#
# Example that shows how to connect to a ROMI from RobotPy
#
# Requirements
# ------------
#
#    # Install https://github.com/wpilibsuite/WPILibPi/releases/download/v2023.2.1/WPILibPi_64_image-v2023.2.1-Romi.zip
#    # on your Raspberry Pi sd card.
#
#    # On Windows, some people prefer to run python 3
#    py -3
#
#    # but sometimes when using Python virtual environments (venv) "py -3" does not run the python associated with
#    # the virtual environment, some people (this author) avoids "py -3", preferring "python"
#
#    # confirm that your python is 3.12 or greater
#    python -VV
#
#    python -m pip install robotpy
#    python -m pip install robotpy-halsim-ws
#
#
# Run the program
# ---------------
#
# To run the program you will need to explicitly use the ws-client option:
#
#    cd to this directory
#    python -m robotpy sync
#
#    power-up the Romi
#    connect to a WiFi network where the romi is on.
#
#    python -m robotpy sim --ws-client
#
# By default the WPILib simulation GUI will be displayed. To disable the display
# you can add the --nogui option
#

import os
import typing
import math

import romi
import wpilib
import commands2
from robotConfig import webserverConstructorOrNone
from commands.arcadedrive import ArcadeDrive
from commands.autonomous_distance import AutonomousDistance
from commands.autonomous_time import AutonomousTime

from robotcontainer import RobotContainer
from utils.signalLogging import SignalWrangler
from utils.signalLogging import log
from utils.crashLogger import CrashLogger
from utils.segmentTimeTracker import SegmentTimeTracker
from utils.robotIdentification import RobotIdentification
from subsystems.drivetrain import Drivetrain

# Uncomment these lines and set the port to the pycharm debugger to use the
# Pycharm debug server to debug this code.

#import pydevd_pycharm
#pydevd_pycharm.settrace('localhost', port=61890, stdoutToServer=True, stderrToServer=True)

# If your ROMI isn't at the default address, set that here
os.environ["HALSIMWS_HOST"] = "10.0.0.2"
os.environ["HALSIMWS_PORT"] = "3300"


class MyRobot(commands2.TimedCommandRobot):
    """
    Command v2 robots are encouraged to inherit from TimedCommandRobot, which
    has an implementation of robotPeriodic which runs the scheduler for you
    """

    autonomousCommand: typing.Optional[commands2.Command] = None
    kCountsPerRevolution = 1440.0
    kWheelDiameterInch = 2.75591
    def robotInit(self) -> None:

        """
        This function is run when the robot is first started up and should be used for any
        initialization code.
        """
        # Since we're defining a bunch of new things here, tell pylint
        # to ignore these instantiations in a method.
        # pylint: disable=attribute-defined-outside-init

        # Instantiate our RobotContainer.  This will perform all our button bindings, and put our
        # autonomous chooser on the dashboard.
        #self.container = RobotContainer()

        #self.drivetrain = Drivetrain()
        self.onboardIO = romi.OnBoardIO(
            romi.OnBoardIO.ChannelMode.INPUT, romi.OnBoardIO.ChannelMode.INPUT
        )

        # Assumes a gamepad plugged into channnel 0
        self.controller = wpilib.Joystick(0)

        # Create SmartDashboard chooser for autonomous routines
        self.chooser = wpilib.SendableChooser()

        """Use this method to define your button->command mappings. Buttons can be created by
        instantiating a :class:`.GenericHID` or one of its subclasses (:class`.Joystick or
        :class:`.XboxController`), and then passing it to a :class:`.JoystickButton`.
        """

        # Default command is arcade drive. This will run unless another command
        # is scheduler over it
        #self.drivetrain.setDefaultCommand(self.getArcadeDriveCommand())

        # Example of how to use the onboard IO
        onboardButtonA = commands2.button.Trigger(self.onboardIO.getButtonAPressed)
        onboardButtonA.onTrue(commands2.PrintCommand("Button A Pressed")).onFalse(
            commands2.PrintCommand("Button A Released")
        )

        # Setup SmartDashboard options
        #self.chooser.setDefaultOption(
        #    "Auto Routine Distance", AutonomousDistance(self.drivetrain)
        #)
        #self.chooser.addOption("Auto Routine Time", AutonomousTime(self.drivetrain))
        #wpilib.SmartDashboard.putData(self.chooser)


        # The Romi has the left and right motors set to
        # PWM channels 0 and 1 respectively
        self.leftMotor = wpilib.Spark(0)
        self.leftMotor.setInverted(True)
        self.rightMotor = wpilib.Spark(1)
        self.leftMotor.setInverted(False)

        # The Romi has onboard encoders that are hardcoded
        # to use DIO pins 4/5 and 6/7 for the left and right
        self.leftEncoder = wpilib.Encoder(4, 5)
        self.rightEncoder = wpilib.Encoder(6, 7)

        # Set up the differential drive controller
        self.drive = wpilib.drive.DifferentialDrive(self.leftMotor, self.rightMotor)

        # Set up the RomiGyro
        self.gyro = romi.RomiGyro()

        # Set up the BuiltInAccelerometer
        self.accelerometer = wpilib.BuiltInAccelerometer()

        # Use inches as unit for encoder distances
        self.leftEncoder.setDistancePerPulse(
            (math.pi * self.kWheelDiameterInch) / self.kCountsPerRevolution
        )
        self.rightEncoder.setDistancePerPulse(
            (math.pi * self.kWheelDiameterInch) / self.kCountsPerRevolution
        )
        self.resetEncoders()

        #self.arcadedrive = self.getArcadeDriveCommand()

        # NOTE: The I/O pin functionality of the 5 exposed I/O pins depends on the hardware "overlay"
        # that is specified when launching the wpilib-ws server on the Romi raspberry pi.
        # By default, the following are available (listed in order from inside of the board to outside):
        # - DIO 8 (mapped to Arduino pin 11, closest to the inside of the board)
        # - Analog In 0 (mapped to Analog Channel 6 / Arduino Pin 4)
        # - Analog In 1 (mapped to Analog Channel 2 / Arduino Pin 20)
        # - PWM 2 (mapped to Arduino Pin 21)
        # - PWM 3 (mapped to Arduino Pin 22)
        #
        # Your subsystem configuration should take the overlays into account


        self.rId = RobotIdentification()
        self.crashLogger = CrashLogger()
        self.stt = SegmentTimeTracker()




    def robotPeriodic(self) -> None:
        """This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
        that you want ran during disabled, autonomous, teleoperated and test.

        This runs after the mode specific periodic functions, but before LiveWindow and
        SmartDashboard integrated updating."""

        # Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
        # commands, running already-scheduled commands, removing finished or interrupted commands,
        # and running subsystem periodic() methods.  This must be called from the robot's periodic
        # block in order for anything in the Command-based framework to work.
        

        leftEncoderCount = self.rightEncoder.get()
        rightEncoderCount = self.leftEncoder.get()
        log("driveLeftEncoder", leftEncoderCount, "count")
        log("driveRightEncoder", rightEncoderCount, "count")
        SignalWrangler().publishPeriodic()

    def disabledInit(self) -> None:
        """This function is called once each time the robot enters Disabled mode."""

    def disabledPeriodic(self) -> None:
        """This function is called periodically when disabled"""

    def autonomousInit(self) -> None:
        """This autonomous runs the autonomous command selected by your RobotContainer class."""
        pass

    def autonomousPeriodic(self) -> None:
        """This function is called periodically during autonomous"""

    def teleopInit(self) -> None:
        # This makes sure that the autonomous stops running when
        # teleop starts running. If you want the autonomous to
        # continue until interrupted by another command, remove
        # this line or comment it out.
        pass

    def teleopPeriodic(self) -> None:
        """This function is called periodically during operator control"""
        forward = self.forward()
        rotation = self.rotation()

        # we might have some confusion: In some modules Drive is: wpilib.drive.DifferentialDrive
        # in some modules drive is: drivetrain.
        self.drive.arcadeDrive(self.forward(), self.rotation())

        log("driveForwardCmd", forward, "ratio")
        log("driveRotationCmd", rotation, "ratio")

        x = self.getGyroAngleX()
        y = self.getGyroAngleY()
        z = self.getGyroAngleZ()
        log("driveGyroAngleX", x, "todo")
        log("driveGyroAngleY", y, "todo")
        log("driveGyroAngleZ", z, "todo")


    def testInit(self) -> None:
        pass

    def getAutonomousCommand(self) -> typing.Optional[commands2.Command]:
        return self.chooser.getSelected()

    #def getArcadeDriveCommand(self) -> ArcadeDrive:
    #    """Use this to pass the teleop command to the main robot class.
    #
    #    :returns: the command to run in teleop
    #    """
    #    return ArcadeDrive(
    #        self.drivetrain,
    #        lambda: -self.controller.getRawAxis(1), # XBox controller in "X" mode, right stick left/right
    #        lambda: self.controller.getRawAxis(4), # XBox controller in "X" mode, left stick forward/back
    #   )
    def resetEncoders(self) -> None:
        """Resets the drive encoders to currently read a position of 0."""
        self.leftEncoder.reset()
        self.rightEncoder.reset()

    def forward(self):
        return -self.controller.getRawAxis(1)

    def rotation(self):
        return self.controller.getRawAxis(4)

    def getAccelX(self) -> float:
        """The acceleration in the X-axis.

        :returns: The acceleration of the Romi along the X-axis in Gs
        """
        return self.accelerometer.getX()

    def getAccelY(self) -> float:
        """The acceleration in the Y-axis.

        :returns: The acceleration of the Romi along the Y-axis in Gs
        """
        return self.accelerometer.getY()

    def getAccelZ(self) -> float:
        """The acceleration in the Z-axis.

        :returns: The acceleration of the Romi along the Z-axis in Gs
        """
        return self.accelerometer.getZ()

    def getGyroAngleX(self) -> float:
        """Current angle of the Romi around the X-axis.

        :returns: The current angle of the Romi in degrees
        """
        return self.gyro.getAngleX()

    def getGyroAngleY(self) -> float:
        """Current angle of the Romi around the Y-axis.

        :returns: The current angle of the Romi in degrees
        """
        return self.gyro.getAngleY()

    def getGyroAngleZ(self) -> float:
        """Current angle of the Romi around the Z-axis.

        :returns: The current angle of the Romi in degrees
        """
        return self.gyro.getAngleZ()


