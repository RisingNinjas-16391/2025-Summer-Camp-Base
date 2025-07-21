package org.firstinspires.ftc.teamcode.commands;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.localization.Pose;

import org.firstinspires.ftc.teamcode.subsystems.Subsystems;
import org.firstinspires.ftc.teamcode.subsystems.drive.Drive;
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake;
import org.firstinspires.ftc.teamcode.subsystems.intake.IntakeConstants;
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
    public static Command blueAuto(Subsystems subsystems) {
        return Commands.sequence(
                DriveCommands.strafeRight(subsystems.drive(), 13),
                DriveCommands.forward(subsystems.drive(), 29),
                Pivot.setPosition(subsystems.pivot(), 0.065),
                DriveCommands.turn(subsystems.drive(), 90),
                Commands.waitSeconds(0.25),
                DriveCommands.forward(subsystems.drive(), 2),
                Pivot.goDown(subsystems.pivot(), subsystems.intake()),
                DriveCommands.backward(subsystems.drive(), 16),
                Intake.setPower(subsystems.intake(), 0).withTimeout(0.5),
                DriveCommands.turn(subsystems.drive(), 90),
                Commands.waitSeconds(0.25),
                Intake.setPower(subsystems.intake(), 1).withTimeout(0.1),
                DriveCommands.forward(subsystems.drive(), 18.5),
                DriveCommands.backward(subsystems.drive(), 27),
                DriveCommands.turn(subsystems.drive(),-90),
                Commands.waitSeconds(0.25),
                Pivot.setPosition(subsystems.pivot(), PivotConstants.LOWBAR),
                DriveCommands.forward(subsystems.drive(),16),
                Pivot.goDown(subsystems.pivot(), subsystems.intake()),
                Intake.setPower(subsystems.intake(),0).withTimeout(0.1),
                DriveCommands.turn(subsystems.drive(),-90),
                Commands.waitSeconds(0.25),
                DriveCommands.backward(subsystems.drive(),30),
                DriveCommands.strafeLeft(subsystems.drive(), 10)
        );
    }

    public static Command redAuto(Subsystems subsystems) {
        return Commands.sequence(
                DriveCommands.strafeLeft(subsystems.drive(), 13),
                DriveCommands.forward(subsystems.drive(), 29),
                Pivot.setPosition(subsystems.pivot(), 0.065),
                DriveCommands.turn(subsystems.drive(), -90),
                Commands.waitSeconds(0.25),
                DriveCommands.forward(subsystems.drive(), 2),
                Pivot.goDown(subsystems.pivot(), subsystems.intake()),
                DriveCommands.backward(subsystems.drive(), 20),
                Intake.setPower(subsystems.intake(), 0).withTimeout(0.5),
                DriveCommands.turn(subsystems.drive(), -90),
                Commands.waitSeconds(0.25),
                Intake.setPower(subsystems.intake(), 1).withTimeout(0.1),
                DriveCommands.forward(subsystems.drive(), 18.5),
                DriveCommands.backward(subsystems.drive(), 27),
                DriveCommands.turn(subsystems.drive(),90),
                Commands.waitSeconds(0.25),
                Pivot.setPosition(subsystems.pivot(), PivotConstants.LOWBAR),
                DriveCommands.forward(subsystems.drive(),20),
                Pivot.goDown(subsystems.pivot(), subsystems.intake()),
                Intake.setPower(subsystems.intake(),0).withTimeout(0.1),
                DriveCommands.turn(subsystems.drive(),90),
                Commands.waitSeconds(0.25),
                DriveCommands.backward(subsystems.drive(),30),
                DriveCommands.strafeRight(subsystems.drive(), 10)
        );
    }
}