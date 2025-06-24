package org.firstinspires.ftc.teamcode.commands;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.localization.Pose;

import org.firstinspires.ftc.teamcode.subsystems.Subsystems;
import org.firstinspires.ftc.teamcode.subsystems.claw.Claw;
import org.firstinspires.ftc.teamcode.subsystems.claw.ClawConstants;
import org.firstinspires.ftc.teamcode.subsystems.pivot.Pivot;
import org.firstinspires.ftc.teamcode.subsystems.pivot.PivotConstants;
import org.firstinspires.ftc.teamcode.subsystems.wrist.Wrist;
import org.firstinspires.ftc.teamcode.subsystems.wrist.WristConstants;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

@Config
public class AutoCommands {
    public static TunablePose pose1Blue = new TunablePose(37, -9, 0);
    public static TunablePose pose1Red = new TunablePose(34, 10, 0);

    public static double scoreForward = 7;
    public static double pickupWait = 0.5;
    public static double scoreBack = 14;
    public static double scoreStrafe = 28;
    public static Command blueAuto(Subsystems subsystems) {
        return Commands.sequence(
                Commands.parallel(
                        Claw.setPosition(subsystems.claw(), () -> ClawConstants.OPEN),
                        Pivot.setPosition(subsystems.pivot(), () -> PivotConstants.FEED_AUTO),
                        Wrist.setPosition(subsystems.wrist(), () -> WristConstants.UPRIGHT)
                ).withTimeout(0),
                DriveCommands.driveToPose(subsystems.drive(), () -> pose1Blue.getPose()),
                Claw.setPosition(subsystems.claw(), () -> ClawConstants.CLOSE).withTimeout(0.0),
                Commands.waitSeconds(() -> pickupWait),
                Pivot.setPosition(subsystems.pivot(), () -> PivotConstants.HIGH_AUTO).withTimeout(0.2),
                DriveCommands.forward(subsystems.drive(), () -> scoreForward),
                Commands.parallel(
                        Pivot.scoreAuto(subsystems.pivot()),
                        Claw.setPosition(subsystems.claw(), () -> ClawConstants.OPEN)).withTimeout(0.0),
                Commands.parallel(
                        DriveCommands.backward(subsystems.drive(), () -> scoreBack),
                        Pivot.setPosition(subsystems.pivot(), () -> PivotConstants.FEED_AUTO).withTimeout(0.5)
                ),
                DriveCommands.strafeRight(subsystems.drive(), () -> scoreStrafe),
                DriveCommands.forward(subsystems.drive(), () -> scoreForward),
                Claw.setPosition(subsystems.claw(), () -> ClawConstants.CLOSE).withTimeout(0.0),
                Commands.waitSeconds(() -> pickupWait),
                Pivot.setPosition(subsystems.pivot(), () -> PivotConstants.HIGH_AUTO).withTimeout(0.2),
                DriveCommands.forward(subsystems.drive(), () -> scoreForward),
                Commands.parallel(
                        Pivot.scoreAuto(subsystems.pivot()),
                        Claw.setPosition(subsystems.claw(), () -> ClawConstants.OPEN)).withTimeout(0.0),
                Commands.parallel(
                        DriveCommands.backward(subsystems.drive(), () -> scoreBack),
                        Pivot.setPosition(subsystems.pivot(), () -> PivotConstants.FEED_AUTO).withTimeout(0.5)
                ),
                DriveCommands.strafeRight(subsystems.drive(), () -> scoreStrafe),
                DriveCommands.forward(subsystems.drive(), () -> scoreForward),
                Claw.setPosition(subsystems.claw(), () -> ClawConstants.CLOSE).withTimeout(0.0),
                Commands.waitSeconds(() -> pickupWait),
                Pivot.setPosition(subsystems.pivot(), () -> PivotConstants.HIGH_AUTO).withTimeout(0.2),
                DriveCommands.forward(subsystems.drive(), () -> scoreForward),
                Commands.parallel(
                        Pivot.scoreAuto(subsystems.pivot()),
                        Claw.setPosition(subsystems.claw(), () -> ClawConstants.OPEN)).withTimeout(0.0),
                Commands.parallel(
                        DriveCommands.backward(subsystems.drive(), () -> scoreBack),
                        Pivot.setPosition(subsystems.pivot(), () -> PivotConstants.FEED_AUTO).withTimeout(0.5)
                ),
                DriveCommands.driveToPose(subsystems.drive(), () -> new Pose(0, 0, 0))

                );
    }

    public static Command redAuto(Subsystems subsystems) {
        return Commands.sequence(
                Commands.parallel(
                        Claw.setPosition(subsystems.claw(), () -> ClawConstants.OPEN),
                        Pivot.setPosition(subsystems.pivot(), () -> PivotConstants.FEED_AUTO),
                        Wrist.setPosition(subsystems.wrist(), () -> WristConstants.UPRIGHT)
                ).withTimeout(0),
                DriveCommands.driveToPose(subsystems.drive(), () -> pose1Red.getPose()),
                Claw.setPosition(subsystems.claw(), () -> ClawConstants.CLOSE).withTimeout(0.0),
                Commands.waitSeconds(() -> pickupWait),
                Pivot.setPosition(subsystems.pivot(), () -> PivotConstants.HIGH_AUTO).withTimeout(0.2),
                DriveCommands.forward(subsystems.drive(), () -> scoreForward),
                Commands.parallel(
                        Pivot.scoreAuto(subsystems.pivot()),
                        Claw.setPosition(subsystems.claw(), () -> ClawConstants.OPEN)).withTimeout(0.0),
                Commands.parallel(
                        DriveCommands.backward(subsystems.drive(), () -> scoreBack),
                        Pivot.setPosition(subsystems.pivot(), () -> PivotConstants.FEED_AUTO).withTimeout(0.5)
                ),
                DriveCommands.strafeLeft(subsystems.drive(), () -> scoreStrafe),
                DriveCommands.forward(subsystems.drive(), () -> scoreForward),
                Claw.setPosition(subsystems.claw(), () -> ClawConstants.CLOSE).withTimeout(0.0),
                Commands.waitSeconds(() -> pickupWait),
                Pivot.setPosition(subsystems.pivot(), () -> PivotConstants.HIGH_AUTO).withTimeout(0.2),
                DriveCommands.forward(subsystems.drive(), () -> scoreForward),
                Commands.parallel(
                        Pivot.scoreAuto(subsystems.pivot()),
                        Claw.setPosition(subsystems.claw(), () -> ClawConstants.OPEN)).withTimeout(0.0),
                Commands.parallel(
                        DriveCommands.backward(subsystems.drive(), () -> scoreBack),
                        Pivot.setPosition(subsystems.pivot(), () -> PivotConstants.FEED_AUTO).withTimeout(0.5)
                ),
                DriveCommands.strafeLeft(subsystems.drive(), () -> scoreStrafe),
                DriveCommands.forward(subsystems.drive(), () -> scoreForward),
                Claw.setPosition(subsystems.claw(), () -> ClawConstants.CLOSE).withTimeout(0.0),
                Commands.waitSeconds(() -> pickupWait),
                Pivot.setPosition(subsystems.pivot(), () -> PivotConstants.HIGH_AUTO).withTimeout(0.2),
                DriveCommands.forward(subsystems.drive(), () -> scoreForward),
                Commands.parallel(
                        Pivot.scoreAuto(subsystems.pivot()),
                        Claw.setPosition(subsystems.claw(), () -> ClawConstants.OPEN)).withTimeout(0.0),
                Commands.parallel(
                        DriveCommands.backward(subsystems.drive(), () -> scoreBack),
                        Pivot.setPosition(subsystems.pivot(), () -> PivotConstants.FEED_AUTO).withTimeout(0.5)
                ),
                DriveCommands.driveToPose(subsystems.drive(), () -> new Pose(0, 0, 0))


        );
    }
}