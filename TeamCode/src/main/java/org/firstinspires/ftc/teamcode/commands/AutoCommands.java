package org.firstinspires.ftc.teamcode.commands;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.localization.Pose;

import org.firstinspires.ftc.teamcode.subsystems.Subsystems;
import org.firstinspires.ftc.teamcode.subsystems.drive.Drive;
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake;
import org.firstinspires.ftc.teamcode.subsystems.pivot.Pivot;
import org.firstinspires.ftc.teamcode.subsystems.pivot.PivotConstants;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

@Config
public class AutoCommands {
    /*
    Available Commands:

    Drivetrain (Replace distance with a number):
    DriveCommands.forward(subsystems.drive(), distance),
    DriveCommands.backward(subsystems.drive(), distance),
    DriveCommands.strafeLeft(subsystems.drive(), distance),
    DriveCommands.strafeRight(subsystems.drive(), distance),

    (Replace angle with a number):
    DriveCommands.turn(subsystems.drive(), angle),

    Pivot (Replace PRESET with a valid Pivot preset):
    Pivot.setPosition(subsystems.pivot(), PivotConstants.PRESET),
    Pivot.setPosition(subsystems.pivot(), () -> PivotConstants.PRESET),
    Pivot.score(subsystems.pivot),

    Claw (Replace position with a number):
    Claw.open(subsystems.claw()),
    Claw.close(subsystems.claw()),

    Claw.setPosition(subsystems.claw(), () -> position);

    Motor Intake (Replace power with a number):
    Intake.setPower(subsystems.intake(), power),
    Intake.setPower(subsystems.intake(), () -> power),

    Servo Intake (Replace power with a number):
    ServoIntake.setPower(subsystems.servoIntake(), power),
    ServoIntake.setPower(subsystems.servoIntake(), () -> power),

    Wrist (Replace PRESET with a valid Wrist preset):
    Wrist.setPosition(subsystems.wrist(), WristPresets.PRESET),
    Wrist.setPosition(subsystems.wrist(), () -> WristPresets.PRESET),

    */

    public static Command redAuto(Subsystems subsystems) {
        return Commands.sequence(
                Pivot.setPosition(subsystems.pivot(), PivotConstants.CLIMB).withTimeout(0.5),
                DriveCommands.driveToPose(subsystems.drive(), new Pose(63,-50)),
                DriveCommands.turn(subsystems.drive(), 180),
                Commands.waitSeconds(1),
                Intake.setPower(subsystems.intake(), -0.725).withTimeout(2),
                DriveCommands.driveToPose(subsystems.drive(), new Pose(40,0)),
                Intake.setPower(subsystems.intake(), 1).withTimeout(2),
                Pivot.setPosition(subsystems.pivot(), PivotConstants.FEED).withTimeout(0.5),
                DriveCommands.forward(subsystems.drive(), 23),
                Intake.setPower(subsystems.intake(), 0),
                Pivot.setPosition(subsystems.pivot(), PivotConstants.CLIMB).withTimeout(0.5),
                DriveCommands.driveToPose(subsystems.drive(), new Pose(63,-50)),
                DriveCommands.turn(subsystems.drive(), 180),
                Commands.waitSeconds(2),
                Intake.setPower(subsystems.intake(), -0.75).withTimeout(2),
                DriveCommands.turn(subsystems.drive(), 180),
                DriveCommands.driveToPose(subsystems.drive(), new Pose(0,0))
        );
    }

    public static Command blueAuto(Subsystems subsystems) {
        return Commands.sequence(
                Pivot.setPosition(subsystems.pivot(), PivotConstants.CLIMB).withTimeout(0.5),
                DriveCommands.driveToPose(subsystems.drive(), new Pose(63,50)),
                DriveCommands.turn(subsystems.drive(), 180),
                //Pivot.setPosition(subsystems.pivot(), PivotConstants.CLIMB),
                Commands.waitSeconds(2),
                Intake.setPower(subsystems.intake(), -0.75).withTimeout(2).withTimeout(0.5),
                DriveCommands.turn(subsystems.drive(), 180),
                DriveCommands.driveToPose(subsystems.drive(), new Pose(40,0)),
                Intake.setPower(subsystems.intake(), 1).withTimeout(10),
                Pivot.setPosition(subsystems.pivot(), PivotConstants.FEED).withTimeout(0.5),
                DriveCommands.forward(subsystems.drive(), 23),
                Intake.setPower(subsystems.intake(), 0),
                Pivot.setPosition(subsystems.pivot(), PivotConstants.CLIMB).withTimeout(0.5),
                DriveCommands.driveToPose(subsystems.drive(), new Pose(63,50)),
                DriveCommands.turn(subsystems.drive(), 180),
                Commands.waitSeconds(2),
                Intake.setPower(subsystems.intake(), -0.75).withTimeout(2),
                DriveCommands.turn(subsystems.drive(), 180),
                DriveCommands.driveToPose(subsystems.drive(), new Pose(0,0))
        );
    }
}