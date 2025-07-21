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
    public static double R = 15;
    public static double F = 40;
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
                DriveCommands.strafeRight(subsystems.drive(), R),
                DriveCommands.forward(subsystems.drive(), F),
                DriveCommands.turn(subsystems.drive(), 90),
                Pivot.setPosition(subsystems.pivot(), PivotConstants.LOW),
                DriveCommands.forward(subsystems.drive(), 4),
                Intake.setPower(subsystems.intake(), -1).withTimeout(0.3),
                Intake.setPower(subsystems.intake(),0).withTimeout(0.1),
                Pivot.score(subsystems.pivot()).alongWith(Intake.setPower(subsystems.intake(), () -> IntakeConstants.INTAKE_POWER).withTimeout(0.2)),
                Commands.waitSeconds(0.25),
                Intake.setPower(subsystems.intake(), () -> IntakeConstants.OUTTAKE_POWER).withTimeout(0.2),
                DriveCommands.backward(subsystems.drive(), 17.5),
                Pivot.setPosition(subsystems.pivot(), PivotConstants.FEEDLOW),
                DriveCommands.turn(subsystems.drive(), 90),
                Intake.setPower(subsystems.intake(), IntakeConstants.INTAKE_POWER).withTimeout(0.2),
                DriveCommands.forward(subsystems.drive(), 27.5),
                Intake.setPower(subsystems.intake(), 0).withTimeout(0),
                Pivot.setPosition(subsystems.pivot(), PivotConstants.LOW),
                DriveCommands.backward(subsystems.drive(), 25),
                DriveCommands.turn(subsystems.drive(), -90),
                DriveCommands.forward(subsystems.drive(), 20),
                Pivot.score(subsystems.pivot()).alongWith(Intake.setPower(subsystems.intake(), () -> IntakeConstants.OUTTAKE_POWER).withTimeout(0.2)),
                Commands.waitSeconds(0.25),
                DriveCommands.backward(subsystems.drive(), 25),
                DriveCommands.strafeLeft(subsystems.drive(), 15),
                DriveCommands.turn(subsystems.drive(), 180),
                DriveCommands.backward(subsystems.drive(), 25.55555555555)
                );

    }

    public static Command redAuto(Subsystems subsystems) {
        return Commands.sequence(
                DriveCommands.strafeLeft(subsystems.drive(), R),
                DriveCommands.forward(subsystems.drive(), F),
                DriveCommands.turn(subsystems.drive(), -90),
                Pivot.setPosition(subsystems.pivot(), PivotConstants.LOW),
                DriveCommands.forward(subsystems.drive(), 3).withTimeout(0.5),
                Intake.setPower(subsystems.intake(), -1).withTimeout(0.0),
                Pivot.score(subsystems.pivot()).alongWith(Intake.setPower(subsystems.intake(), () -> IntakeConstants.INTAKE_POWER).withTimeout(0.2)),
                Commands.waitSeconds(0.25),
                Intake.setPower(subsystems.intake(), () -> IntakeConstants.OUTTAKE_POWER).withTimeout(0.2),
                DriveCommands.backward(subsystems.drive(), 19),
                Pivot.setPosition(subsystems.pivot(), PivotConstants.FEEDLOW + 0.01),
                DriveCommands.turn(subsystems.drive(), -90),
                Intake.setPower(subsystems.intake(), IntakeConstants.INTAKE_POWER).withTimeout(0.2),
                DriveCommands.forward(subsystems.drive(), 30),
                Intake.setPower(subsystems.intake(), 0).withTimeout(0),
                Pivot.setPosition(subsystems.pivot(), PivotConstants.LOW),
                DriveCommands.backward(subsystems.drive(), 25),
                DriveCommands.turn(subsystems.drive(), 90),
                Intake.setPower(subsystems.intake(), -0.5).withTimeout(0),
                DriveCommands.forward(subsystems.drive(), 21),
                Pivot.score(subsystems.pivot()).alongWith(Intake.setPower(subsystems.intake(), () -> IntakeConstants.OUTTAKE_POWER).withTimeout(0.2)),
                Commands.waitSeconds(0.25),
                DriveCommands.backward(subsystems.drive(), 10),
                DriveCommands.strafeRight(subsystems.drive(), 25),
                DriveCommands.turn(subsystems.drive(), 180),
                DriveCommands.backward(subsystems.drive(), 15)

        );
    }
}