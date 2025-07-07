package org.firstinspires.ftc.teamcode.commands;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathBuilder;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;

import org.firstinspires.ftc.teamcode.subsystems.Subsystems;
import org.firstinspires.ftc.teamcode.subsystems.drive.Drive;
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake;
import org.firstinspires.ftc.teamcode.subsystems.intake.IntakeConstants;
import org.firstinspires.ftc.teamcode.subsystems.pivot.Pivot;
import org.firstinspires.ftc.teamcode.subsystems.pivot.PivotConstants;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

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

    public static double BLUE_AUTO_FORWARD_DISTANCE = 64.0;
    public static double BLUE_AUTO_STRAFE_DISTANCE = 50.0;
    public static double BLUE_AUTO_INITIAL_PIECE_POSITION_X = 0.0;
    public static double BLUE_AUTO_INITIAL_PIECE_POSITION_Y = 0.0;

    public static double RED_AUTO_FORWARD_DISTANCE = 62.0;
    public static double RED_AUTO_STRAFE_DISTANCE = -46.0;
    public static double RED_AUTO_INITIAL_PIECE_POSITION_X = 72.0;
    public static double RED_AUTO_INITIAL_PIECE_POSITION_Y = 12.0;
    public static double RED_AUTO_INITIAL_PIECE_HEADING = 90.0;

    public static Command blueAuto(Subsystems subsystems) {
        return Commands.sequence(
                Pivot.resetPosition(subsystems.pivot()),
                DriveCommands.forward(subsystems.drive(), 36.0),
                DriveCommands.driveToPose(subsystems.drive(), Pose::new)
        );
    }

    public static Command redAuto(Subsystems subsystems) {
        return Commands.sequence(
                Pivot.resetPosition(subsystems.pivot()),
                DriveCommands.forward(subsystems.drive(), 36.0),
                DriveCommands.driveToPose(subsystems.drive(), Pose::new)
        );
    }

//    public static Command blueAuto(Subsystems subsystems) {
//        return Commands.sequence(
//                Pivot.resetPosition(subsystems.pivot()),
//                DriveCommands.driveToPose(subsystems.drive(), () -> new Pose(
//                        BLUE_AUTO_INITIAL_PIECE_POSITION_X,
//                        BLUE_AUTO_INITIAL_PIECE_POSITION_Y,
//                        -45
//                )),
//                new ParallelCommandGroup(
//                        DriveCommands.driveToPose(subsystems.drive(), () -> new Pose(
//                                BLUE_AUTO_FORWARD_DISTANCE,
//                                BLUE_AUTO_STRAFE_DISTANCE
//                        )),
//                        Commands.waitSeconds(2)
//                                .andThen(Pivot.setPosition(
//                                        subsystems.pivot(),
//                                        () -> PivotConstants.HIGH)
//                                )),
//                Intake.setPower(subsystems.intake(), -1.0).withTimeout(2),
//                Intake.setPower(subsystems.intake(), 0.0).withTimeout(.2),
//                DriveCommands.driveToPose(subsystems.drive(), Pose::new)
//        );
//    }
//
//    public static Command redAuto(Subsystems subsystems) {
//        return Commands.sequence(
//                Commands.parallel(
//                        Commands.sequence(
//                                Pivot.resetPosition(subsystems.pivot()),
//                                Pivot.setPosition(subsystems.pivot(), () -> PivotConstants.HIGH)
//                        ),
//                        DriveCommands.driveToPose(subsystems.drive(), () -> new Pose(
//                                RED_AUTO_FORWARD_DISTANCE,
//                                RED_AUTO_STRAFE_DISTANCE
//                        ))
//                ),
//                Intake.setPower(subsystems.intake(), () -> IntakeConstants.OUTTAKE).withTimeout(0.5),
//                Intake.setPower(subsystems.intake(), () -> 0.0).withTimeout(0.2),
//                new ParallelCommandGroup(
//                        Commands.sequence(
//                                Commands.waitSeconds(0.2),
//                                DriveCommands.driveToPose(subsystems.drive(), () -> new Pose(
//                                        RED_AUTO_INITIAL_PIECE_POSITION_X,
//                                        RED_AUTO_INITIAL_PIECE_POSITION_Y,
//                                        RED_AUTO_INITIAL_PIECE_HEADING * Math.PI / 180.0
//                                ))
//                        ),
//                        Pivot.setPosition(subsystems.pivot(), () -> PivotConstants.FEED),
//                        Intake.setPower(subsystems.intake(), () -> IntakeConstants.INTAKE).withTimeout(0.2)
//                ),
//                Commands.waitSeconds(3.0),
//                Intake.setPower(subsystems.intake(), () -> 0.0).withTimeout(0.2),
//                new ParallelCommandGroup(
//                        DriveCommands.driveToPose(subsystems.drive(), () -> new Pose(
//                                RED_AUTO_FORWARD_DISTANCE,
//                                RED_AUTO_STRAFE_DISTANCE
//                        )),
//                        Commands.sequence(
//                                Commands.waitSeconds(0.5),
//                                Pivot.setPosition(subsystems.pivot(), () -> PivotConstants.HIGH)
//                        )
//                ),
//                Intake.setPower(subsystems.intake(), () -> IntakeConstants.OUTTAKE).withTimeout(0.2),
//                Intake.setPower(subsystems.intake(), () -> 0.0).withTimeout(0.2),
//                new SequentialCommandGroup(
//                        DriveCommands.driveToPose(subsystems.drive(), Pose::new),
//                        Pivot.resetPosition(subsystems.pivot())
//                )
//        );
//    }
}
