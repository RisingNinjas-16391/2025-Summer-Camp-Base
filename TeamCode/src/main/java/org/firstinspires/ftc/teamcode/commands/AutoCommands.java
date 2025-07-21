package org.firstinspires.ftc.teamcode.commands;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.localization.Pose;

import org.firstinspires.ftc.teamcode.subsystems.Subsystems;
import org.firstinspires.ftc.teamcode.subsystems.claw.Claw;
import org.firstinspires.ftc.teamcode.subsystems.drive.Drive;
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
                Pivot.setPosition(subsystems.pivot(), PivotConstants.LOW),
                DriveCommands.strafeRight(subsystems.drive(), 16),


                DriveCommands.forward(subsystems.drive(), 38),

                DriveCommands.turn(subsystems.drive(), 90),
                DriveCommands.forward(subsystems.drive(), 9),
                Claw.open(subsystems.claw()).withTimeout(0.5),
                DriveCommands.backward(subsystems.drive(), 20),
                DriveCommands.turn(subsystems.drive(), 90),
                Pivot.setPosition(subsystems.pivot(), PivotConstants.FEED),
                DriveCommands.forward(subsystems.drive(), 30),
                Claw.close(subsystems.claw()).withTimeout(0.5),
                DriveCommands.backward(subsystems.drive(), 28),
                Pivot.setPosition(subsystems.pivot(), PivotConstants.LOW),
                DriveCommands.turn(subsystems.drive(), -90),
                DriveCommands.forward(subsystems.drive(),24),
                Claw.open(subsystems.claw()).withTimeout(0.5),
                Pivot.setPosition(subsystems.pivot(), PivotConstants.FEED),
                DriveCommands.backward(subsystems.drive(),12),
                DriveCommands.turn(subsystems.drive(), -90),
                DriveCommands.backward(subsystems.drive(), 20),
                DriveCommands.strafeLeft(subsystems.drive(), 20)



                );
    }

    public static Command redAuto(Subsystems subsystems) {
        return Commands.sequence(
                Pivot.setPosition(subsystems.pivot(), PivotConstants.LOW-0.01),
                DriveCommands.strafeLeft(subsystems.drive(), 16),


                DriveCommands.forward(subsystems.drive(), 38),

                DriveCommands.turn(subsystems.drive(), -90),
                DriveCommands.forward(subsystems.drive(), 9),
                Claw.open(subsystems.claw()).withTimeout(0.5),
                DriveCommands.backward(subsystems.drive(), 23),
                DriveCommands.turn(subsystems.drive(), -90),
                Pivot.setPosition(subsystems.pivot(), PivotConstants.FEED),
                DriveCommands.forward(subsystems.drive(), 33),
                Claw.close(subsystems.claw()).withTimeout(0.5),
                DriveCommands.backward(subsystems.drive(), 23),
                Pivot.setPosition(subsystems.pivot(), PivotConstants.LOW-0.01),
                DriveCommands.turn(subsystems.drive(), 90),
                DriveCommands.forward(subsystems.drive(),24),
                Claw.open(subsystems.claw()).withTimeout(0.5),
                Pivot.setPosition(subsystems.pivot(), PivotConstants.FEED),
                DriveCommands.backward(subsystems.drive(),12),
                DriveCommands.turn(subsystems.drive(), 90),
                DriveCommands.backward(subsystems.drive(), 20),
                DriveCommands.strafeRight(subsystems.drive(), 20)
        );
    }

}