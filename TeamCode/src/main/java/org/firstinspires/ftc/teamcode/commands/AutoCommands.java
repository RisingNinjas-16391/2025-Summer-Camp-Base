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
    public static double Y = -37.5;
    public static double X = 58;
    public static double X2 = 65;
    public static double Y2 = -9;
    public static double S = 0.75;

    public static double s = 0.7;
    public static double redhoop = 222;
    public static double redball = -45;

    public static Command blueAuto(Subsystems subsystems) {
        return Commands.sequence(
                DriveCommands.driveToPose(subsystems.drive(), () -> new Pose(X, Y, Math.toRadians(140))),
                Shooter.setPower(subsystems.shooter(), () -> S).withTimeout(1.5),
                Intake.setPower(subsystems.intake(), () -> 0.5).withTimeout(1),
                Shooter.setPower(subsystems.shooter(), () -> 0).withTimeout(0),
                DriveCommands.driveToPose(subsystems.drive(), () -> new Pose(X2, Y2, Math.toRadians(35))),
                Intake.setPower(subsystems.intake(), () -> 0).withTimeout(1),
                Shooter.setPower(subsystems.shooter(), () -> -0.25).withTimeout(0),
                Intake.setPower(subsystems.intake(), () -> -0.25).withTimeout(0.5),
                Shooter.setPower(subsystems.shooter(), () -> 0).withTimeout(0),
                Intake.setPower(subsystems.intake(), () -> 0).withTimeout(0),
                DriveCommands.driveToPose(subsystems.drive(), () -> new Pose(X, Y, Math.toRadians(140))),
                Shooter.setPower(subsystems.shooter(), () -> S).withTimeout(1.5),
                Intake.setPower(subsystems.intake(), () -> 0.25).withTimeout(1),
                Intake.setPower(subsystems.intake(), () -> 0).withTimeout(0),
                Shooter.setPower(subsystems.shooter(), () -> 0).withTimeout(0),
                DriveCommands.driveToPose(subsystems.drive(), () -> new Pose(0, 0, Math.toRadians(0)))

        );
    }

    public static Command redAuto(Subsystems subsystems) {
        return Commands.sequence(

                DriveCommands.driveToPose(subsystems.drive(), () -> new Pose(X, 40, Math.toRadians(redhoop))),
                Shooter.setPower(subsystems.shooter(), () -> s).withTimeout(1.5),
                Intake.setPower(subsystems.intake(), () -> 0.5).withTimeout(1),
                Shooter.setPower(subsystems.shooter(), () -> 0).withTimeout(0),
                DriveCommands.driveToPose(subsystems.drive(), () -> new Pose(X2, -Y2, Math.toRadians(redball))),
                Commands.waitSeconds(1),
                Intake.setPower(subsystems.intake(), () -> 0).withTimeout(1),
                Shooter.setPower(subsystems.shooter(), () -> -0.25).withTimeout(0),
                Intake.setPower(subsystems.intake(), () -> -0.25).withTimeout(0.5),
                Shooter.setPower(subsystems.shooter(), () -> 0).withTimeout(0),
                Intake.setPower(subsystems.intake(), () -> 0).withTimeout(0),
                DriveCommands.driveToPose(subsystems.drive(), () -> new Pose(X, -Y, Math.toRadians(redhoop))),
                Shooter.setPower(subsystems.shooter(), () -> S).withTimeout(1.5),
                Intake.setPower(subsystems.intake(), () -> 0.25).withTimeout(1),
                Intake.setPower(subsystems.intake(), () -> 0).withTimeout(0),
                Shooter.setPower(subsystems.shooter(), () -> 0).withTimeout(0),
                DriveCommands.driveToPose(subsystems.drive(), () -> new Pose(0, 0, Math.toRadians(0)))


        );
    }
}