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
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

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

    public static Command blueAuto(Subsystems subsystems) {
        return Commands.sequence(
                new ParallelCommandGroup(
                DriveCommands.driveToPose(subsystems.drive(), () -> new Pose(
                        BLUE_AUTO_FORWARD_DISTANCE,
                        BLUE_AUTO_STRAFE_DISTANCE
                )),
                Pivot.setPosition(subsystems.pivot(), () -> PivotConstants.HIGH)),
                Intake.setPower(subsystems.intake(), () -> 1.0),
                DriveCommands.driveToPose(subsystems.drive(), Pose::new)
        );
    }

    public static Command redAuto(Subsystems subsystems) {
        return Commands.sequence(
                DriveCommands.forward(subsystems.drive(), 10)
        );
    }
}