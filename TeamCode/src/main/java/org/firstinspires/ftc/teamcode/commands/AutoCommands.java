package org.firstinspires.ftc.teamcode.commands;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.localization.Pose;

import org.firstinspires.ftc.teamcode.subsystems.Subsystems;
import org.firstinspires.ftc.teamcode.subsystems.claw.Claw;
import org.firstinspires.ftc.teamcode.subsystems.claw.ClawConstants;
import org.firstinspires.ftc.teamcode.subsystems.drive.Drive;
import org.firstinspires.ftc.teamcode.subsystems.pivot.Pivot;
import org.firstinspires.ftc.teamcode.subsystems.pivot.PivotConstants;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

@Config
public class AutoCommands {
    public static double AUTO_FORWARD_DISTANCE = 30.0;
    public static double AUTO_STRAFE_DISTANCE = 11.0;
    public static double AUTO_lSTRAFE_DISTANCE = 0.25;
    public static double AUTO_Drive_DISTANCE = 12;
    public static double AUTO_DriveBackwards_Distance = 2;
    public static double AUTO_DRIVESCORE_DISTANCE = 4;
    public static double AUTO_SECOND_STRAFE = 28;
    public static Command blueAuto(Subsystems subsystems) {
        return Commands.sequence(
                DriveCommands.strafeRight(subsystems.drive(), () -> AUTO_STRAFE_DISTANCE ),
                DriveCommands.forward(subsystems.drive(), () -> AUTO_FORWARD_DISTANCE),
                Pivot.setPosition(subsystems.pivot(), () -> PivotConstants.FEED).withTimeout(0.2),
                Claw.setPosition(subsystems.claw(), () -> ClawConstants.OPEN).withTimeout(0.2),
                DriveCommands.strafeLeft(subsystems.drive(), () -> AUTO_lSTRAFE_DISTANCE),
                DriveCommands.forward(subsystems.drive(), () -> AUTO_Drive_DISTANCE),
                Claw.setPosition(subsystems.claw(), () -> ClawConstants.CLOSE).withTimeout(0.2),
                DriveCommands.backward(subsystems.drive(), () -> AUTO_DriveBackwards_Distance),
                Pivot.setPosition(subsystems.pivot(), () -> PivotConstants.HIGH).withTimeout(0.2),
                DriveCommands.forward(subsystems.drive(), () -> AUTO_DRIVESCORE_DISTANCE),
                Pivot.setPosition(subsystems.pivot(), () -> PivotConstants.LOW).withTimeout(1.0),
                Claw.setPosition(subsystems.claw(), () -> ClawConstants.OPEN).withTimeout(0.2),
                DriveCommands.backward(subsystems.drive(), () -> 2),
                Pivot.setPosition(subsystems.pivot(), () -> PivotConstants.HIGH).withTimeout(1.0),
                DriveCommands.strafeRight(subsystems.drive(), () -> AUTO_SECOND_STRAFE),
                Pivot.setPosition(subsystems.pivot(), () -> PivotConstants.FEED).withTimeout(0.2),
                DriveCommands. forward(subsystems.drive(), () -> 4),
                Claw.setPosition(subsystems.claw(), () -> ClawConstants.CLOSE).withTimeout(0.2),
                Pivot.setPosition(subsystems.pivot(), () -> PivotConstants.HIGH).withTimeout(1.0),
                DriveCommands.forward(subsystems.drive(), () -> AUTO_DRIVESCORE_DISTANCE),
                Claw.setPosition(subsystems.claw(), () -> ClawConstants.OPEN).withTimeout(0.2),
                DriveCommands.backward(subsystems.drive(), () -> AUTO_DRIVESCORE_DISTANCE+3),
                Pivot.setPosition(subsystems.pivot(), () -> -0.12).withTimeout(0.2),
                DriveCommands.driveToPose(subsystems.drive(), Pose::new)
        );
    }

    public static Command redAuto(Subsystems subsystems) {
        return Commands.sequence(
              Pivot.setPosition(subsystems.pivot(),() -> PivotConstants.FEED).withTimeout(0.2),
                DriveCommands.strafeLeft(subsystems.drive(), () -> AUTO_lSTRAFE_DISTANCE+10.50),
                DriveCommands.forward(subsystems.drive(), () -> AUTO_FORWARD_DISTANCE),
                Claw.setPosition(subsystems.claw(), () -> ClawConstants.OPEN).withTimeout(0.2),
                DriveCommands.forward(subsystems.drive(), () -> 7),
                Claw.setPosition(subsystems.claw(), () -> ClawConstants.CLOSE).withTimeout(0.5),
                Pivot.setPosition(subsystems.pivot(), () -> PivotConstants.HIGH).withTimeout(0.2),
                DriveCommands.forward(subsystems.drive(), () -> AUTO_DRIVESCORE_DISTANCE),
                Claw.setPosition(subsystems.claw(), () -> ClawConstants.OPEN).withTimeout(0.4),
                DriveCommands.backward(subsystems.drive(), () -> 5),
                Claw.setPosition(subsystems.claw(), () -> ClawConstants.CLOSE).withTimeout(0.2),
                DriveCommands.strafeLeft(subsystems.drive(), () -> AUTO_lSTRAFE_DISTANCE+10.50),
                DriveCommands.forward(subsystems.drive(), () -> AUTO_FORWARD_DISTANCE),
                Claw.setPosition(subsystems.claw(), () -> ClawConstants.OPEN).withTimeout(0.2),
                DriveCommands.forward(subsystems.drive(), () -> 7),
                Claw.setPosition(subsystems.claw(), () -> ClawConstants.CLOSE).withTimeout(0.5),
                Pivot.setPosition(subsystems.pivot(), () -> PivotConstants.HIGH).withTimeout(0.2),
                DriveCommands.forward(subsystems.drive(), () -> AUTO_DRIVESCORE_DISTANCE),
                Claw.setPosition(subsystems.claw(), () -> ClawConstants.OPEN).withTimeout(0.4),
                DriveCommands.backward(subsystems.drive(), () -> 5),
                Pivot.setPosition(subsystems.pivot(), () -> PivotConstants.LOW),
                DriveCommands.driveToPose(subsystems.drive(), Pose::new)

        );
    }
}