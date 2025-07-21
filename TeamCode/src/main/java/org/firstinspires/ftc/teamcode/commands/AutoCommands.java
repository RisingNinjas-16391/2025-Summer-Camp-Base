package org.firstinspires.ftc.teamcode.commands;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.localization.Pose;

import org.firstinspires.ftc.teamcode.subsystems.Subsystems;
import org.firstinspires.ftc.teamcode.subsystems.drive.Drive;
import org.firstinspires.ftc.teamcode.subsystems.pivot.Pivot;
import org.firstinspires.ftc.teamcode.subsystems.pivot.PivotConstants;
import org.firstinspires.ftc.teamcode.subsystems.claw.Claw;
import org.firstinspires.ftc.teamcode.subsystems.claw.ClawConstants;
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
                DriveCommands.strafeRight(subsystems.drive(), 13.5),
                DriveCommands.forward(subsystems.drive(), 34),
                DriveCommands.turn(subsystems.drive(), 90).alongWith(Pivot.setPosition(subsystems.pivot(), () -> PivotConstants.LOW)),
                DriveCommands.forward(subsystems.drive(), 8),
                Claw.open(subsystems.claw()).withTimeout(.8),
                DriveCommands.backward(subsystems.drive(), 23),
                DriveCommands.turn(subsystems.drive(), 90).alongWith(Pivot.setPosition(subsystems.pivot(), () -> PivotConstants.FEED)),
                DriveCommands.forward(subsystems.drive(), 25),
                Claw.close(subsystems.claw()).withTimeout(.5),
                DriveCommands.backward(subsystems.drive(), 34),
                DriveCommands.turn(subsystems.drive(), -90).alongWith(Pivot.setPosition(subsystems.pivot(), () -> PivotConstants.LOW)),
                DriveCommands.forward(subsystems.drive(), 27),
                Claw.open(subsystems.claw()).withTimeout(.8),
                DriveCommands.backward(subsystems.drive(), 11).alongWith(Pivot.setPosition(subsystems.pivot(), () -> PivotConstants.FEED)),
                DriveCommands.driveToPose(subsystems.drive(), Pose::new)
        );
    }

    public static Command redAuto(Subsystems subsystems) {
        return Commands.sequence(
                DriveCommands.strafeLeft(subsystems.drive(), 13.5).alongWith(Claw.close(subsystems.claw()).withTimeout(.6)),
                DriveCommands.forward(subsystems.drive(), 29),
                DriveCommands.turn(subsystems.drive(), -90).alongWith(Pivot.setPosition(subsystems.pivot(), () -> PivotConstants.LOW)),
                Commands.waitSeconds(.5),
                DriveCommands.forward(subsystems.drive(), 9).withTimeout(1),
                Claw.open(subsystems.claw()).withTimeout(.6),
                Pivot.setPosition(subsystems.pivot(), () -> PivotConstants.FEED),
                DriveCommands.backward(subsystems.drive(), 27).andThen(Commands.waitSeconds(.4)).andThen(DriveCommands.turn(subsystems.drive(), -90)),
                DriveCommands.forward(subsystems.drive(), 20),
                Commands.waitSeconds(1),
                DriveCommands.forward(subsystems.drive(), 5),
                Claw.close(subsystems.claw()).withTimeout(.5),
                DriveCommands.backward(subsystems.drive(), 28),
                Commands.waitSeconds(.5),
                DriveCommands.turn(subsystems.drive(), 90).alongWith(Pivot.setPosition(subsystems.pivot(), () -> PivotConstants.LOW)),
                DriveCommands.forward(subsystems.drive(), 28),
                Claw.open(subsystems.claw()).withTimeout(.8),
                DriveCommands.backward(subsystems.drive(), 10).alongWith((Pivot.setPosition(subsystems.pivot(), () -> PivotConstants.FEED))),
                DriveCommands.driveToPose(subsystems.drive(), Pose::new)
                );
    }
}