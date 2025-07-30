package org.firstinspires.ftc.teamcode.commands;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.localization.Pose;

import org.firstinspires.ftc.teamcode.subsystems.Subsystems;
import org.firstinspires.ftc.teamcode.subsystems.drive.Drive;
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake;
import org.firstinspires.ftc.teamcode.subsystems.pivot.Pivot;
import org.firstinspires.ftc.teamcode.subsystems.pivot.PivotConstants;
import org.firstinspires.ftc.teamcode.subsystems.shooter.Shooter;

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
                DriveCommands.driveToPose(subsystems.drive(), () -> new Pose(-61, 42, Math.toRadians(-43))),
                Shooter.setPower(subsystems.shooter(), -0.76).withTimeout(2),
                Intake.setPower(subsystems.intake(), -1).withTimeout(1),
                Shooter.setPower(subsystems.shooter(), 0).withTimeout(0),
                Intake.setPower(subsystems.intake(), 0).withTimeout(0),

                Intake.setPower(subsystems.intake(), 1).withTimeout(0),
                DriveCommands.driveToPose(subsystems.drive(), () -> new Pose(-65, 2, Math.toRadians(-135))),
                Intake.setPower(subsystems.intake(), -0.1).withTimeout(0.1),
                Intake.setPower(subsystems.intake(), 0).withTimeout(0),

                DriveCommands.driveToPose(subsystems.drive(), () -> new Pose(-61, 42, Math.toRadians(-35))),
                Shooter.setPower(subsystems.shooter(), -.75).withTimeout(4),
                Intake.setPower(subsystems.intake(), -1).withTimeout(1),
                Shooter.setPower(subsystems.shooter(), 0).withTimeout(0),

                DriveCommands.driveToPose(subsystems.drive(), () -> new Pose(0, 0, Math.toRadians(0)))

//                DriveCommands.forward(subsystems.drive(), distance),

                );
    }

    public static Command redAuto(Subsystems subsystems) {
        return Commands.sequence(
                DriveCommands.driveToPose(subsystems.drive(), () -> new Pose(-58, -44, Math.toRadians(43))),
                Shooter.setPower(subsystems.shooter(), -0.85).withTimeout(2),
                Intake.setPower(subsystems.intake(), -1).withTimeout(1),
                Shooter.setPower(subsystems.shooter(), 0).withTimeout(0),

                Intake.setPower(subsystems.intake(), -1).withTimeout(0),
                DriveCommands.driveToPose(subsystems.drive(), () -> new Pose(-62, -9, Math.toRadians(150))),
                Intake.setPower(subsystems.intake(), 0).withTimeout(2),

                DriveCommands.driveToPose(subsystems.drive(), () -> new Pose(-58, -44, Math.toRadians(43))),
                Shooter.setPower(subsystems.shooter(), -1).withTimeout(4),
                Intake.setPower(subsystems.intake(), -1).withTimeout(1),
                Shooter.setPower(subsystems.shooter(), 0).withTimeout(0),

                DriveCommands.driveToPose(subsystems.drive(), () -> new Pose(0, 0, Math.toRadians(0)))
        );
    }
}