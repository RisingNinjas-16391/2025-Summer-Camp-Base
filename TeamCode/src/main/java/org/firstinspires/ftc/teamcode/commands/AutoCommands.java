package org.firstinspires.ftc.teamcode.commands;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.localization.Pose;

import org.firstinspires.ftc.robotcore.internal.ftdi.eeprom.FT_EEPROM_2232D;
import org.firstinspires.ftc.teamcode.subsystems.Subsystems;
import org.firstinspires.ftc.teamcode.subsystems.drive.Drive;
import org.firstinspires.ftc.teamcode.subsystems.pivot.Pivot;
import org.firstinspires.ftc.teamcode.subsystems.pivot.PivotConstants;
import org.firstinspires.ftc.teamcode.subsystems.servo_intake.ServoIntake;
import org.firstinspires.ftc.teamcode.subsystems.servo_intake.ServoIntakeConstants;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

@Config
public class AutoCommands {
    public static Command blueAuto(Subsystems subsystems) {
        return Commands.sequence(
            Pivot.setPosition(subsystems.pivot(), () -> PivotConstants.FEED).alongWith(ServoIntake.setPower(subsystems.servoIntake(), () -> -1).withTimeout(4)),
            DriveCommands.strafeRight(subsystems.drive(), () -> 9.5),
            DriveCommands.forward(subsystems.drive(), () -> 42.4),
            Commands.waitSeconds(.2),
            ServoIntake.setPower(subsystems.servoIntake(), () -> 0).withTimeout(.5),
            //Commands.waitSeconds(.2),
            Pivot.setPosition(subsystems.pivot(),()-> PivotConstants.HIGH).withTimeout(2),
            DriveCommands.forward(subsystems.drive(), () -> 3),
            Commands.waitSeconds(.8),
            Pivot.setPosition(subsystems.pivot(),()-> PivotConstants.FEED).withTimeout(2).andThen(ServoIntake.setPower(subsystems.servoIntake(), () -> 1).withTimeout(1)).alongWith(DriveCommands.backward(subsystems.drive(), () -> 2)),
            DriveCommands.backward(subsystems.drive(), () -> 5).alongWith(ServoIntake.setPower(subsystems.servoIntake(), () -> 0).withTimeout(2)),
            DriveCommands.strafeRight(subsystems.drive(), () -> 27.8),
            Commands.waitSeconds(.2),
            DriveCommands.forward(subsystems.drive(), () -> 5),
            Commands.waitSeconds(.9),
            ServoIntake.setPower(subsystems.servoIntake(), () -> -1).withTimeout(2),
            Commands.waitSeconds(.53),
            Pivot.setPosition(subsystems.pivot(),()-> PivotConstants.HIGH).withTimeout(1).alongWith(ServoIntake.setPower(subsystems.servoIntake(), () -> 0).withTimeout(.5)),
            DriveCommands.forward(subsystems.drive(), () -> 2.8),
            Commands.waitSeconds(.3),
            Pivot.setPosition(subsystems.pivot(),()-> PivotConstants.FEED).withTimeout(2).andThen(ServoIntake.setPower(subsystems.servoIntake(), () -> 1).withTimeout(1)).alongWith(DriveCommands.backward(subsystems.drive(), () -> 2)),
            DriveCommands.backward(subsystems.drive(), () -> 4).alongWith(Pivot.setPosition(subsystems.pivot(),()-> PivotConstants.initialPosition)),
            DriveCommands.driveToPose(subsystems.drive(), () -> new Pose()


                ));
    }

    public static Command redAuto(Subsystems subsystems) {
        return Commands.sequence(
                Pivot.setPosition(subsystems.pivot(), () -> PivotConstants.FEED).alongWith(ServoIntake.setPower(subsystems.servoIntake(), () -> -1).withTimeout(.8)),
                DriveCommands.strafeLeft(subsystems.drive(), () -> 11),
                DriveCommands.forward(subsystems.drive(), () -> 39.7),
                Commands.waitSeconds(.2),
                ServoIntake.setPower(subsystems.servoIntake(), () -> 0).withTimeout(.4),
                Commands.waitSeconds(.4),
                Pivot.setPosition(subsystems.pivot(),()-> PivotConstants.HIGH).withTimeout(1).alongWith(ServoIntake.setPower(subsystems.servoIntake(), () -> 0).withTimeout(.5)),
                DriveCommands.forward(subsystems.drive(), () -> 3.2),
                Commands.waitSeconds(.5),
                Pivot.setPosition(subsystems.pivot(),()-> PivotConstants.FEED).withTimeout(2).andThen(ServoIntake.setPower(subsystems.servoIntake(), () -> 1).withTimeout(1)).alongWith(DriveCommands.backward(subsystems.drive(), () -> 2)),
                DriveCommands.backward(subsystems.drive(), () -> 4),
                ServoIntake.setPower(subsystems.servoIntake(), () -> 0).withTimeout(.5),
                DriveCommands.strafeLeft(subsystems.drive(), () -> 26.5).alongWith(ServoIntake.setPower(subsystems.servoIntake(), () -> -1).withTimeout(1)),
                Commands.waitSeconds(.8),
                DriveCommands.forward(subsystems.drive(), () -> 3.5),
                ServoIntake.setPower(subsystems.servoIntake(), () -> 0).withTimeout(.5),
                Commands.waitSeconds(.8),
                Pivot.setPosition(subsystems.pivot(),()-> PivotConstants.HIGH).withTimeout(2),
                DriveCommands.forward(subsystems.drive(), () -> 2.5),
                Pivot.setPosition(subsystems.pivot(),()-> PivotConstants.FEED).withTimeout(2).andThen(ServoIntake.setPower(subsystems.servoIntake(), () -> 1).withTimeout(1)).alongWith(DriveCommands.backward(subsystems.drive(), () -> 2)),
                DriveCommands.backward(subsystems.drive(), () -> 4).alongWith(Pivot.setPosition(subsystems.pivot(),()-> PivotConstants.initialPosition)),
                DriveCommands.driveToPose(subsystems.drive(), () -> new Pose()


        ));

    }
}