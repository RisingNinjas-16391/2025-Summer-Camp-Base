package org.firstinspires.ftc.teamcode.commands;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.localization.Pose;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.subsystems.Subsystems;
import org.firstinspires.ftc.teamcode.subsystems.drive.Drive;
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake;
import org.firstinspires.ftc.teamcode.subsystems.pivot.Pivot;
import org.firstinspires.ftc.teamcode.subsystems.pivot.PivotConstants;
import org.firstinspires.ftc.teamcode.subsystems.servo_intake.ServoIntake;
import org.firstinspires.ftc.teamcode.subsystems.servo_intake.ServoIntakeConstants;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

@Config
public class AutoCommands {
    public static double RED_AUTO_FORWARD_DISTANCE = 39.0;
    public static double BLUE_AUTO_FORWARD_DISTANCE = 42;
    public static double RED_AUTO_STRAFE_DISTANCE = 11.0;
    public static double BLUE_AUTO_STRAFE_DISTANCE = 8.5;
    public static double AUTO_SCORE_DISTANCE = 3.0;
    public static double AUTO_REVERSE_DISTANCE = 2.0;
    public static double CONE_PRESET_DISTANCE = 27.125;

    public static double INTAKE_TIME = 1.5;
    public static double PIVOT_TIMEOUT = 0.5;
    public static double DEFAULT_TIMEOUT = 0.2;

    public static SequentialCommandGroup score(Subsystems subsystems) {
        return new SequentialCommandGroup(
                Pivot.setPosition(subsystems.pivot(), () -> PivotConstants.LOW).withTimeout(PIVOT_TIMEOUT),
                ServoIntake.setPower(subsystems.servoIntake(), () -> ServoIntakeConstants.INTAKE).withTimeout(DEFAULT_TIMEOUT),
                Commands.waitSeconds(INTAKE_TIME),
                ServoIntake.setPower(subsystems.servoIntake(), () -> ServoIntakeConstants.setpoint).withTimeout(DEFAULT_TIMEOUT),
                Pivot.setPosition(subsystems.pivot(), () -> PivotConstants.HIGH).withTimeout(PIVOT_TIMEOUT),
                DriveCommands.forward(subsystems.drive(), () -> AUTO_SCORE_DISTANCE),
                new ParallelCommandGroup(
                        Pivot.setPosition(subsystems.pivot(), () -> PivotConstants.MIDDLE).withTimeout(PIVOT_TIMEOUT),
                        new SequentialCommandGroup(
                                ServoIntake.setPower(subsystems.servoIntake(), () -> ServoIntakeConstants.OUTTAKE).withTimeout(DEFAULT_TIMEOUT),
                                Commands.waitSeconds(INTAKE_TIME),
                                ServoIntake.setPower(subsystems.servoIntake(), () -> ServoIntakeConstants.setpoint).withTimeout(DEFAULT_TIMEOUT)
                        )
                ),
                DriveCommands.backward(subsystems.drive(), () -> AUTO_SCORE_DISTANCE)
        );
    }

    public static Command blueAuto(Subsystems subsystems) {
        return Commands.sequence(
                Pivot.setPosition(subsystems.pivot(), () -> PivotConstants.LOW).withTimeout(PIVOT_TIMEOUT),
                DriveCommands.driveToPose(subsystems.drive(), () -> new Pose(BLUE_AUTO_FORWARD_DISTANCE, -BLUE_AUTO_STRAFE_DISTANCE)),
                score(subsystems),
                new ParallelCommandGroup(
                        DriveCommands.driveToPose(subsystems.drive(), () -> new Pose(
                                        BLUE_AUTO_FORWARD_DISTANCE - AUTO_REVERSE_DISTANCE,
                                        -BLUE_AUTO_STRAFE_DISTANCE - CONE_PRESET_DISTANCE,
                                        subsystems.drive().getPose().getHeading()
                                )
                        ),
                        Pivot.setPosition(subsystems.pivot(), () -> PivotConstants.LOW).withTimeout(PIVOT_TIMEOUT)
                ),
                DriveCommands.forward(subsystems.drive(), () -> AUTO_REVERSE_DISTANCE),
                score(subsystems),
                new ParallelCommandGroup(
                        DriveCommands.driveToPose(subsystems.drive(), () -> new Pose(
                                        BLUE_AUTO_FORWARD_DISTANCE - AUTO_REVERSE_DISTANCE,
                                        -BLUE_AUTO_STRAFE_DISTANCE - (CONE_PRESET_DISTANCE * 2),
                                        subsystems.drive().getPose().getHeading()
                                )
                        ),
                        Pivot.setPosition(subsystems.pivot(), () -> PivotConstants.LOW).withTimeout(PIVOT_TIMEOUT)
                ),
                DriveCommands.forward(subsystems.drive(), () -> AUTO_REVERSE_DISTANCE),
                score(subsystems),
                DriveCommands.driveToPose(subsystems.drive(), Pose::new)
        );
    }

    public static Command redAuto(Subsystems subsystems) {
        return Commands.sequence(
                Pivot.setPosition(subsystems.pivot(), () -> PivotConstants.LOW).withTimeout(PIVOT_TIMEOUT),
                DriveCommands.driveToPose(subsystems.drive(), () -> new Pose(RED_AUTO_FORWARD_DISTANCE, RED_AUTO_STRAFE_DISTANCE)),
                score(subsystems),
                new ParallelCommandGroup(
                        DriveCommands.driveToPose(subsystems.drive(), () -> new Pose(
                                        RED_AUTO_FORWARD_DISTANCE - AUTO_REVERSE_DISTANCE,
                                        RED_AUTO_STRAFE_DISTANCE + CONE_PRESET_DISTANCE,
                                        subsystems.drive().getPose().getHeading()
                                )
                        ),
                        Pivot.setPosition(subsystems.pivot(), () -> PivotConstants.LOW).withTimeout(PIVOT_TIMEOUT)
                ),
                DriveCommands.forward(subsystems.drive(), () -> AUTO_REVERSE_DISTANCE),
                score(subsystems),
                new ParallelCommandGroup(
                        DriveCommands.driveToPose(subsystems.drive(), () -> new Pose(
                                        RED_AUTO_FORWARD_DISTANCE - AUTO_REVERSE_DISTANCE,
                                        RED_AUTO_STRAFE_DISTANCE + (CONE_PRESET_DISTANCE * 2),
                                        subsystems.drive().getPose().getHeading()
                                )
                        ),
                        Pivot.setPosition(subsystems.pivot(), () -> PivotConstants.LOW).withTimeout(PIVOT_TIMEOUT)
                ),
                DriveCommands.forward(subsystems.drive(), () -> AUTO_REVERSE_DISTANCE),
                score(subsystems),
                DriveCommands.driveToPose(subsystems.drive(), Pose::new)
        );
    }

    // This one works
    /*
    public static Command redAuto(Subsystems subsystems) {
        return Commands.sequence(
                Pivot.setPosition(subsystems.pivot(), () -> PivotConstants.LOW).withTimeout(PIVOT_TIMEOUT),
                DriveCommands.driveToPose(subsystems.drive(), () -> new Pose(AUTO_FORWARD_DISTANCE, AUTO_STRAFE_DISTANCE)),
                score(subsystems),
                DriveCommands.strafeLeft(subsystems.drive(), () -> CONE_PRESET_DISTANCE),
                score(subsystems),
                DriveCommands.strafeLeft(subsystems.drive(), () -> CONE_PRESET_DISTANCE),
                score(subsystems),
                DriveCommands.driveToPose(subsystems.drive(), Pose::new)
        );
    }
    */
}