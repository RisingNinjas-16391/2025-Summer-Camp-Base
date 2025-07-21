package org.firstinspires.ftc.teamcode.commands;

import static org.firstinspires.ftc.teamcode.subsystems.pivot.PivotConstants.LOW_SCORE;

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
                DriveCommands.forward(subsystems.drive(), 10),
                DriveCommands.strafeRight(subsystems.drive(),14),
                DriveCommands.forward(subsystems.drive(), 29),
                DriveCommands.turn(subsystems.drive(), 90).withTimeout(1.5),
                Pivot.setPosition(subsystems.pivot(), LOW_SCORE),
                DriveCommands.forward(subsystems.drive(), 4),
                Pivot.score(subsystems.pivot(), 0.1),
                DriveCommands.backward(subsystems.drive(), 17.5),
                Pivot.setPosition(subsystems.pivot(), PivotConstants.LOW_FEED),
                DriveCommands.turn(subsystems.drive(), 90).withTimeout(1.5),
                Intake.setPower(subsystems.intake(), IntakeConstants.INTAKE_POWER).withTimeout(0),
                DriveCommands.forward(subsystems.drive(), 21),
                DriveCommands.backward(subsystems.drive(), 28),
                Intake.setPower(subsystems.intake(), 0).withTimeout(0),
                DriveCommands.turn(subsystems.drive(), -90).withTimeout(1.5),
                Pivot.setPosition(subsystems.pivot(), LOW_SCORE),
                DriveCommands.forward(subsystems.drive(), 20),
                Pivot.score(subsystems.pivot(), 0.1),
                DriveCommands.backward(subsystems.drive(), 5),
                DriveCommands.turn(subsystems.drive(), -105).withTimeout(1.5),
                DriveCommands.backward(subsystems.drive(), 30)
















        );
    }

    public static Command redAuto(Subsystems subsystems) {
        return Commands.sequence(
                DriveCommands.forward(subsystems.drive(), 10),
                DriveCommands.strafeLeft(subsystems.drive(),14),
                DriveCommands.forward(subsystems.drive(), 29),
                DriveCommands.turn(subsystems.drive(), -90).withTimeout(1.5),
                Pivot.setPosition(subsystems.pivot(), LOW_SCORE),
                DriveCommands.forward(subsystems.drive(), 4),
                Pivot.score(subsystems.pivot(), 0.1),
                DriveCommands.backward(subsystems.drive(), 21),
                Pivot.setPosition(subsystems.pivot(), PivotConstants.LOW_FEED),
                DriveCommands.turn(subsystems.drive(), -90).withTimeout(1.5),
                Intake.setPower(subsystems.intake(), IntakeConstants.INTAKE_POWER).withTimeout(0),
                DriveCommands.forward(subsystems.drive(), 26),
                DriveCommands.backward(subsystems.drive(), 30),
                Intake.setPower(subsystems.intake(), 0).withTimeout(0),
                DriveCommands.turn(subsystems.drive(), 90).withTimeout(1.5),
                Pivot.setPosition(subsystems.pivot(), LOW_SCORE),
                DriveCommands.forward(subsystems.drive(), 22),
                Pivot.score(subsystems.pivot(), 0.1),
                DriveCommands.backward(subsystems.drive(), 5),
                DriveCommands.turn(subsystems.drive(), 110).withTimeout(1.5),
                DriveCommands.backward(subsystems.drive(), 50)
        );
    }
}