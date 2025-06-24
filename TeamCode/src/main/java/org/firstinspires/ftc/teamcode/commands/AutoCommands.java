package org.firstinspires.ftc.teamcode.commands;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.localization.Pose;

import org.firstinspires.ftc.teamcode.subsystems.Subsystems;
import org.firstinspires.ftc.teamcode.subsystems.claw.Claw;
import org.firstinspires.ftc.teamcode.subsystems.claw.ClawConstants;
import org.firstinspires.ftc.teamcode.subsystems.pivot.Pivot;
import org.firstinspires.ftc.teamcode.subsystems.pivot.PivotConstants;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

@Config
public class AutoCommands {
    public static double forwardDistance = 41.8;
    public static double strafeRightDistance = 9.8;
    public static double strafeRightDistance2 = 27;
    public static double feedDistance = 1.0;
    public static double scoreDistance = 3.5;
    public static double strafeLeftDistanceRed = 9.8;

    public static double strafeLeftDistanceRed2 = 27.125;
    public static double forwardDistanceRed = 50.2;



    public static Command blueAuto(Subsystems subsystems) {
        return Commands.sequence(
                Pivot.setPosition(subsystems.pivot(), () -> PivotConstants.FEED),
                Claw.setPosition(subsystems.claw(), () -> ClawConstants.OPEN),
                DriveCommands.forward(subsystems.drive(), () -> forwardDistance),
                DriveCommands.strafeRight(subsystems.drive(), () -> strafeRightDistance),
                DriveCommands.forward(subsystems.drive(), () -> feedDistance),
                Claw.setPosition(subsystems.claw(), () -> ClawConstants.CLOSE),
                Commands.waitSeconds(.5),
                Pivot.setPosition(subsystems.pivot(), () -> PivotConstants.HIGH),
                DriveCommands.forward(subsystems.drive(), () -> scoreDistance),
                Claw.setPosition(subsystems.claw(), () -> ClawConstants.OPEN),
                Commands.waitSeconds(.3),
                DriveCommands.backward(subsystems.drive(), () -> scoreDistance + feedDistance),
                Pivot.setPosition(subsystems.pivot(), () -> PivotConstants.FEED),
                DriveCommands.strafeRight(subsystems.drive(), () -> strafeRightDistance2),
                DriveCommands.forward(subsystems.drive(), () -> feedDistance),
                Claw.setPosition(subsystems.claw(), () -> ClawConstants.CLOSE),
                Commands.waitSeconds(.5),
                Pivot.setPosition(subsystems.pivot(), () -> PivotConstants.HIGH),
                DriveCommands.forward(subsystems.drive(), () -> scoreDistance),
                Claw.setPosition(subsystems.claw(), () -> ClawConstants.OPEN),
                Commands.waitSeconds(.3),
                DriveCommands.backward(subsystems.drive(), () -> scoreDistance + feedDistance),
                Pivot.setPosition(subsystems.pivot(), () -> PivotConstants.FEED),
                DriveCommands.strafeRight(subsystems.drive(), () -> strafeRightDistance2 + 1),
                DriveCommands.forward(subsystems.drive(), () -> feedDistance),
                Claw.setPosition(subsystems.claw(), () -> ClawConstants.CLOSE),
                Commands.waitSeconds(.5),
                Pivot.setPosition(subsystems.pivot(), () -> PivotConstants.HIGH),
                DriveCommands.forward(subsystems.drive(), () -> scoreDistance + .1),
                Claw.setPosition(subsystems.claw(), () -> ClawConstants.OPEN),
                Commands.waitSeconds(.3),
                DriveCommands.backward(subsystems.drive(), () ->2),
                DriveCommands.strafeLeft(subsystems.drive(), () -> 65),
                DriveCommands.backward(subsystems.drive(), () -> 38)
        );
    }

    public static Command redAuto(Subsystems subsystems) {
        return Commands.sequence(
                Pivot.setPosition(subsystems.pivot(), () -> PivotConstants.FEED),
                Claw.setPosition(subsystems.claw(), () -> ClawConstants.OPEN),
                DriveCommands.strafeLeft(subsystems.drive(), () -> strafeLeftDistanceRed),
                DriveCommands.forward(subsystems.drive(), () -> forwardDistanceRed + 1.8).withTimeout(1),
                DriveCommands.forward(subsystems.drive(), () -> feedDistance),
                Claw.setPosition(subsystems.claw(), () -> ClawConstants.CLOSE),
                Commands.waitSeconds(1),
                Pivot.setPosition(subsystems.pivot(), () -> PivotConstants.HIGH),
                DriveCommands.forward(subsystems.drive(), () -> scoreDistance +1),
                Claw.setPosition(subsystems.claw(), () -> ClawConstants.OPEN),
                Commands.waitSeconds(.3),
                DriveCommands.backward(subsystems.drive(), () -> scoreDistance + feedDistance),
                Pivot.setPosition(subsystems.pivot(), () -> PivotConstants.FEED),
                DriveCommands.strafeLeft(subsystems.drive(), () -> strafeLeftDistanceRed2),
                DriveCommands.forward(subsystems.drive(), () -> feedDistance + 1),
                Claw.setPosition(subsystems.claw(), () -> ClawConstants.CLOSE),
                Commands.waitSeconds(.5),
                Pivot.setPosition(subsystems.pivot(), () -> PivotConstants.HIGH),
                DriveCommands.forward(subsystems.drive(), () -> scoreDistance + 1),
                Claw.setPosition(subsystems.claw(), () -> ClawConstants.OPEN),
                Commands.waitSeconds(.3),
                DriveCommands.backward(subsystems.drive(), () -> scoreDistance + 1 + feedDistance + .5),
                Pivot.setPosition(subsystems.pivot(), () -> PivotConstants.FEED),
                DriveCommands.strafeLeft(subsystems.drive(), () -> strafeLeftDistanceRed2 +1),
                DriveCommands.forward(subsystems.drive(), () -> feedDistance + .8),
                Claw.setPosition(subsystems.claw(), () -> ClawConstants.CLOSE),
                Commands.waitSeconds(.5),
                Pivot.setPosition(subsystems.pivot(), () -> PivotConstants.HIGH),
                DriveCommands.forward(subsystems.drive(), () -> scoreDistance - .5),
                Claw.setPosition(subsystems.claw(), () -> ClawConstants.OPEN),
                Commands.waitSeconds(.3),
                DriveCommands.backward(subsystems.drive(), () ->2),
                DriveCommands.strafeRight(subsystems.drive(), () -> 65),
                DriveCommands.backward(subsystems.drive(), () -> 44)
        );
    }
}