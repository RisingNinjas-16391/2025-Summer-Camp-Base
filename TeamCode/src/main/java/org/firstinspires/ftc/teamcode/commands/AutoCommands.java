package org.firstinspires.ftc.teamcode.commands;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.localization.Pose;

import org.firstinspires.ftc.teamcode.commands.auto.TunablePose;
import org.firstinspires.ftc.teamcode.subsystems.Subsystems;
import org.firstinspires.ftc.teamcode.subsystems.drive.Drive;
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake;
import org.firstinspires.ftc.teamcode.subsystems.pivot.Pivot;
import org.firstinspires.ftc.teamcode.subsystems.pivot.PivotConstants;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

@Config
public class AutoCommands {
    public static TunablePose pose1 = new TunablePose(48, 74, -90);
    public static TunablePose pose2 = new TunablePose(0,0, -90);
    public static double ball2 = 35;
    public static double backward1 = -5;
    public static double move1 = 10;
    public static double turn1 = 30;
    public static double backward2 = 5;
    public static TunablePose pose3 = new TunablePose(48, -74, 90);
    public static TunablePose pose4 = new TunablePose(0, 0, 90);
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
                DriveCommands.setPose(subsystems.drive(), () -> new Pose(0, 0, Math.toRadians(90))),
                Commands.parallel(
                        DriveCommands.driveToPose(subsystems.drive(), pose3::getPose),
                        Pivot.setPosition(subsystems.pivot(), () -> PivotConstants.HIGH2).withTimeout(.3)
                ),
                Intake.setPower(subsystems.intake(),-0.8).withTimeout(.7),
                DriveCommands.backward(subsystems.drive(), backward1),
                Intake.setPower(subsystems.intake(), 0.8).withTimeout(.5),
                DriveCommands.turn(subsystems.drive(), 90),
                Pivot.setPosition(subsystems.pivot(), () -> PivotConstants.FEED).withTimeout(.3),
                DriveCommands.forward(subsystems.drive(), ball2-2),
                DriveCommands.turn(subsystems.drive(), turn1),
                DriveCommands.forward(subsystems.drive(), move1),
                DriveCommands.backward(subsystems.drive(), move1),
                Intake.setPower(subsystems.intake(),0.5).withTimeout(0.1),
                DriveCommands.backward(subsystems.drive(),8),
                DriveCommands.turn(subsystems.drive(), -turn1),
                Commands.parallel(
                        DriveCommands.backward(subsystems.drive(), ball2-7),
                        Pivot.setPosition(subsystems.pivot(), () -> PivotConstants.HIGH2).withTimeout(.3)
                ),
                DriveCommands.turn(subsystems.drive(), -90),
                DriveCommands.backward(subsystems.drive(), backward2).withTimeout(.5),
                Intake.setPower(subsystems.intake(),-0.8).withTimeout(.7),
                Commands.parallel(
                        DriveCommands.driveToPose(subsystems.drive(),pose4::getPose),
                        Pivot.setPosition(subsystems.pivot(), () -> PivotConstants.FEED).withTimeout(.3)
                )

        );
    }

    public static Command redAuto(Subsystems subsystems) {
        return Commands.sequence(
                DriveCommands.setPose(subsystems.drive(), () -> new Pose(0, 0, Math.toRadians(-90))),
                Commands.parallel(
                        DriveCommands.driveToPose(subsystems.drive(), pose1::getPose),
                        Pivot.setPosition(subsystems.pivot(), () -> PivotConstants.HIGH2).withTimeout(.3)
                ),
                Intake.setPower(subsystems.intake(),-0.6).withTimeout(.7),
                DriveCommands.backward(subsystems.drive(), backward1),
                Intake.setPower(subsystems.intake(), 1).withTimeout(.5),
                DriveCommands.turn(subsystems.drive(), -90),
                Pivot.setPosition(subsystems.pivot(), () -> PivotConstants.FEED).withTimeout(.3),
                DriveCommands.forward(subsystems.drive(), ball2),
                DriveCommands.turn(subsystems.drive(), -turn1),
                DriveCommands.forward(subsystems.drive(), move1),
                DriveCommands.backward(subsystems.drive(), move1),
                Intake.setPower(subsystems.intake(),0.5).withTimeout(0.1),
                DriveCommands.backward(subsystems.drive(),8),
                DriveCommands.turn(subsystems.drive(), turn1),
                Commands.parallel(
                        DriveCommands.backward(subsystems.drive(), ball2-7),
                        Pivot.setPosition(subsystems.pivot(), () -> PivotConstants.HIGH2).withTimeout(.3)
                ),
                DriveCommands.turn(subsystems.drive(), 90),
                DriveCommands.backward(subsystems.drive(), backward2),
                Intake.setPower(subsystems.intake(),-0.6).withTimeout(.7),
                Commands.parallel(
                        DriveCommands.driveToPose(subsystems.drive(),pose2::getPose),
                        Pivot.setPosition(subsystems.pivot(), () -> PivotConstants.FEED).withTimeout(.3)
                )

        );
    }
}