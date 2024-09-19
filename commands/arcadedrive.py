#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#

import typing
import commands2
from subsystems.drivetrain import Drivetrain
from utils.signalLogging import log


class ArcadeDrive(commands2.Command):
    def __init__(
        self,
        drive_train: Drivetrain,
        forward: typing.Callable[[], float],
        rotation: typing.Callable[[], float],
    ) -> None:
        """Creates a new ArcadeDrive. This command will drive your robot according to the speed supplier
        lambdas. This command does not terminate.

        :param drivetrain:  The drivetrain subsystem on which this command will run
        :param forward:     Callable supplier of forward/backward speed
        :param rotation:    Callable supplier of rotational speed
        """
        super().__init__()

        self.drive_train = drive_train
        self.forward = forward
        self.rotation = rotation

        self.addRequirements(self.drive_train)

    def execute(self) -> None:
        forward = self.forward()
        rotation = self.rotation()
        self.drive_train.arcadeDrive(self.forward(), self.rotation())

        log("driveForwardCmd", forward, "ratio")
        log("driveRotationCmd", rotation, "ratio")

        x = self.drive_train.getGyroAngleX()
        y = self.drive_train.getGyroAngleY()
        z = self.drive_train.getGyroAngleZ()
        log("driveGyroAngleX", x, "todo")
        log("driveGyroAngleY", y, "todo")
        log("driveGyroAngleZ", z, "todo")