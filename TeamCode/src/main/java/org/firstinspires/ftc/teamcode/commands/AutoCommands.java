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
//RED VARIABLES
    public static double RED_AUTO_FORWARD_DISTANCE = 71.0;
    public static double RED_AUTO_STRAFE_DISTANCE = 46.0;
    public static double RED_FIRST_BACK = 8;
    public static double RED_MOVE_SIDE_BALL = 53;
    public static double RED_MOVE_FOWARD_BALL = 7;
    public static double RED_SECOND_BACK = 6;
    public static double RED_MOVE2_SIDE_BALL = 52;

    //BLUE VARIABLES
    public static double BLUE_AUTO_FORWARD_DISTANCE = -67.0;
    public static double BLUE_AUTO_STRAFE_DISTANCE = 51.0;
    public static double BLUE_FIRST_BACK = 7;
    public static double BLUE_MOVE_SIDE_BALL = -43;
    public static double BLUE_MOVE_FOWARD_BALL = 10;
    public static double BLUE_SECOND_BACK = 5;
    public static double BLUE_MOVE2_SIDE_BALL = -46;
    public static Command blueAuto(Subsystems subsystems) {
        return Commands.sequence(
                DriveCommands.setPose(subsystems.drive(), () -> new Pose(0, 0, Math.toRadians(90))),
                DriveCommands.driveToPose(subsystems.drive(), () -> new Pose(
                        BLUE_AUTO_STRAFE_DISTANCE,
                        BLUE_AUTO_FORWARD_DISTANCE,
                        Math.toRadians(90)
                )),
                Pivot.setPosition(subsystems.pivot(), PivotConstants.HIGH).withTimeout(5),
                Intake.setPower(subsystems.intake(), 0.68).withTimeout(2),
                Intake.setPower(subsystems.intake(), 0).withTimeout(.1),
                Pivot.setPosition(subsystems.pivot(), PivotConstants.FEED),
                DriveCommands.forward(subsystems.drive(), BLUE_FIRST_BACK),
                DriveCommands.turn(subsystems.drive(), 180),
                DriveCommands.strafeLeft(subsystems.drive(), BLUE_MOVE_SIDE_BALL),
                Intake.setPower(subsystems.intake(), -1).withTimeout(2),
                DriveCommands.forward(subsystems.drive(), BLUE_MOVE_FOWARD_BALL),
                Intake.setPower(subsystems.intake(), 0).withTimeout(.1),
                DriveCommands.backward(subsystems.drive(), BLUE_SECOND_BACK),
                DriveCommands.strafeRight(subsystems.drive(), BLUE_MOVE2_SIDE_BALL),
                DriveCommands.turn(subsystems.drive(), 180),
                Pivot.setPosition(subsystems.pivot(), PivotConstants.HIGH),
                Intake.setPower(subsystems.intake(), 0.72).withTimeout(2),
                Intake.setPower(subsystems.intake(), 0).withTimeout(.1),
//                DriveCommands.strafeLeft(subsystems.drive(), 46),
//                DriveCommands.backward(subsystems.drive(), 71)
                DriveCommands.driveToPose(subsystems.drive(), Pose::new),
                Pivot.setPosition(subsystems.pivot(), PivotConstants.FEED)
                );
        }


    public static Command redAuto(Subsystems subsystems) {
        return Commands.sequence(
                DriveCommands.setPose(subsystems.drive(), () -> new Pose(0, 0, Math.toRadians(-90))),
//                DriveCommands.strafeRight(subsystems.drive(), 46),
//                DriveCommands.forward(subsystems.drive(), 71),
                DriveCommands.driveToPose(subsystems.drive(), () -> new Pose(
                        RED_AUTO_STRAFE_DISTANCE,
                        RED_AUTO_FORWARD_DISTANCE,
                        Math.toRadians(-90)
                )),
                Pivot.setPosition(subsystems.pivot(), PivotConstants.HIGH).withTimeout(2),
                Intake.setPower(subsystems.intake(), 0.65).withTimeout(2),
                Intake.setPower(subsystems.intake(), 0).withTimeout(.1),
                Pivot.setPosition(subsystems.pivot(), PivotConstants.FEED),
                DriveCommands.forward(subsystems.drive(), RED_FIRST_BACK),
                DriveCommands.turn(subsystems.drive(), 180),
                DriveCommands.strafeLeft(subsystems.drive(), RED_MOVE_SIDE_BALL),
                Intake.setPower(subsystems.intake(), -1).withTimeout(2),
                DriveCommands.forward(subsystems.drive(), RED_MOVE_FOWARD_BALL),
                Intake.setPower(subsystems.intake(), 0).withTimeout(.1),
                DriveCommands.backward(subsystems.drive(), RED_SECOND_BACK),
                DriveCommands.strafeRight(subsystems.drive(), RED_MOVE2_SIDE_BALL),
                DriveCommands.turn(subsystems.drive(), 180),
                Pivot.setPosition(subsystems.pivot(), PivotConstants.HIGH),
                Intake.setPower(subsystems.intake(), 0.68).withTimeout(2),
                Intake.setPower(subsystems.intake(), 0).withTimeout(.1),
//                DriveCommands.strafeLeft(subsystems.drive(), 46),
//                DriveCommands.backward(subsystems.drive(), 71)
                DriveCommands.driveToPose(subsystems.drive(), Pose::new),
                Pivot.setPosition(subsystems.pivot(), PivotConstants.FEED)
        );
    }
}