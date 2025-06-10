package org.firstinspires.ftc.teamcode.commands;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathBuilder;

import org.ejml.dense.row.FMatrixComponent;
import org.firstinspires.ftc.teamcode.lib.controller.SquIDController;
import org.firstinspires.ftc.teamcode.subsystems.drive.Drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class DriveCommands {

    private DriveCommands() {}

    /**
     * Field relative drive command using two joysticks (controlling linear and angular velocities).
     */
    public static Command joystickDrive(
            Drive drive,
            DoubleSupplier xSupplier,
            DoubleSupplier ySupplier,
            DoubleSupplier omegaSupplier) {
        SlewRateLimiter xLimiter = new SlewRateLimiter(2, -10, 0);
        SlewRateLimiter yLimiter = new SlewRateLimiter(2, -10, 0);
        SlewRateLimiter rotLimiter = new SlewRateLimiter(2, -10, 0);

        return Commands.runOnce(drive::startTeleopDrive, drive).andThen(Commands.run(() -> drive.drive(
                () -> Math.signum(xSupplier.getAsDouble()) * xLimiter.calculate(Math.abs(xSupplier.getAsDouble())),
                () -> Math.signum(ySupplier.getAsDouble()) * yLimiter.calculate(Math.abs(ySupplier.getAsDouble())),
                () -> Math.signum(omegaSupplier.getAsDouble()) * rotLimiter.calculate(Math.abs(omegaSupplier.getAsDouble())))));
    }

    /**
     * Field relative drive command using joystick for linear control and PID for angular control.
     * Possible use cases include snapping to an angle, aiming at a vision target, or controlling
     * absolute rotation with a joystick.
     */
    public static Command joystickDriveAtAngle(
            Drive drive,
            DoubleSupplier xSupplier,
            DoubleSupplier ySupplier,
            DoubleSupplier rotationSupplier) {

        // Create PID controller
        SquIDController angleController =
                new SquIDController(
                        0.1);

        angleController.enableContinuousInput(-Math.PI, Math.PI);

        // Construct command
        return Commands.runOnce(drive::startTeleopDrive, drive).andThen(
                Commands.run(
                        () -> {
                            // Calculate angular speed
                            double omega =
                                    angleController.calculate(
                                            drive.getPose().getHeading(), rotationSupplier.getAsDouble());

                            drive.drive(
                                    xSupplier,
                                    ySupplier,
                                    () -> omega);
                        },
                        drive));
    }

    public static Command driveToPose(Drive drive, Supplier<Pose> pose) {
        return Drive.followPath(
                drive,
                () -> new PathBuilder()
                        .addPath(new Path(new BezierLine(drive.getPose(), pose.get())))
                        .setLinearHeadingInterpolation(drive.getPose().getHeading(), pose.get().getHeading())
                        .build());
    }

    public static Command driveToPose(Drive drive, Supplier<Pose> pose, Command command, DoubleSupplier commandActivationPoint) {
        return Drive.followPath(
                drive,
                () -> new PathBuilder()
                        .addPath(new Path(new BezierLine(drive.getPose(), pose.get())))
                        .setLinearHeadingInterpolation(drive.getPose().getHeading(), pose.get().getHeading())
                        .addParametricCallback(commandActivationPoint.getAsDouble(), command::schedule)
                        .build());
    }

    public static Command setPose(Drive drive, Supplier<Pose> pose) {
        return Commands.runOnce(() -> drive.setPose(pose.get()), drive);
    }

    public static Command forward(Drive drive, DoubleSupplier distance) {
        return Drive.followPath(
                drive,
                () -> new PathBuilder()
                        .addPath(new Path(new BezierLine(
                                drive.getPose(),
                                new Pose(
                                        drive.getPose().getX() + Math.cos(drive.getPose().getHeading()) * distance.getAsDouble(),
                                        drive.getPose().getY() + Math.sin(drive.getPose().getHeading()) * distance.getAsDouble(),
                                        drive.getPose().getHeading()))))
                        .setLinearHeadingInterpolation(drive.getPose().getHeading(), drive.getPose().getHeading())
                        .build());
    }

    public static Command backward(Drive drive, DoubleSupplier distance) {
        return Drive.followPath(
                drive,
                () -> new PathBuilder()
                        .addPath(new Path(new BezierLine(
                                drive.getPose(),
                                new Pose(
                                        drive.getPose().getX() - Math.cos(drive.getPose().getHeading()) * distance.getAsDouble(),
                                        drive.getPose().getY() - Math.sin(drive.getPose().getHeading()) * distance.getAsDouble(),
                                        drive.getPose().getHeading()))))
                        .setLinearHeadingInterpolation(drive.getPose().getHeading(), drive.getPose().getHeading())
                        .build());
    }

    public static Command strafeLeft(Drive drive, DoubleSupplier distance) {
        return Drive.followPath(
                drive,
                () -> new PathBuilder()
                        .addPath(new Path(new BezierLine(
                                drive.getPose(),
                                new Pose(
                                        drive.getPose().getX() - Math.sin(drive.getPose().getHeading()) * distance.getAsDouble(),
                                        drive.getPose().getY() + Math.cos(drive.getPose().getHeading()) * distance.getAsDouble(),
                                        drive.getPose().getHeading()))))
                        .setLinearHeadingInterpolation(drive.getPose().getHeading(), drive.getPose().getHeading())
                        .build());
    }

    public static Command strafeRight(Drive drive, DoubleSupplier distance) {
        return Drive.followPath(
                drive,
                () -> new PathBuilder()
                        .addPath(new Path(new BezierLine(
                                drive.getPose(),
                                new Pose(
                                        drive.getPose().getX() + Math.sin(drive.getPose().getHeading()) * distance.getAsDouble(),
                                        drive.getPose().getY() - Math.cos(drive.getPose().getHeading()) * distance.getAsDouble(),
                                        drive.getPose().getHeading()))))
                        .setLinearHeadingInterpolation(drive.getPose().getHeading(), drive.getPose().getHeading())
                        .build());
    }


    public static Command turn(Drive drive, DoubleSupplier angle) {
        return Drive.followPath(
                drive,
                () -> new PathBuilder()
                        .addPath(new Path(new BezierLine(drive.getPose(), drive.getPose())))
                        .setLinearHeadingInterpolation(drive.getPose().getHeading(), drive.getPose().getHeading() + Math.toRadians(angle.getAsDouble()))
                        .build());
    }

    public static double signSquare(double num) {
        return num * num * Math.signum(num);
    }
}
