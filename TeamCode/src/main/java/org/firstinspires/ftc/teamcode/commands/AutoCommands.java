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
    public static double BackDistance = 27.9;
    public static double BackDistance2 = 12;
    public static double ReverseFeed = 0.84;
    public static double Strafe1 = 11;
    public static double Strafe2 = 27;
    public static double HIGH2 = PivotConstants.HIGH+.1;


    public static Command blueAuto(Subsystems subsystems) {
        return Commands.sequence(
                Commands.parallel(
                        Claw.setPosition(subsystems.claw(),()-> ClawConstants.OPEN).withTimeout(.1),
                        DriveCommands.driveToPose(subsystems.drive(),()->new Pose(-31, 9, 0)),
                        (Pivot.setPosition(subsystems.pivot(), ()-> ReverseFeed)),
                        (Wrist.setPosition(subsystems.wrist(), ()-> WristConstants.FLIPPED)).withTimeout(.1)
                ).withTimeout(1.7),
                Claw.setPosition(subsystems.claw(),()-> ClawConstants.CLOSE).withTimeout(.4),
                (Pivot.setPosition(subsystems.pivot(), ()-> PivotConstants.HIGH)).withTimeout(.5),
                DriveCommands.backward(subsystems.drive(),()->BackDistance2),
                Pivot.setPosition(subsystems.pivot(),()->HIGH2).withTimeout(.2),
                Claw.setPosition(subsystems.claw(),()-> ClawConstants.OPEN).withTimeout(.5),
                DriveCommands.forward(subsystems.drive(),()->BackDistance2+2),
                Pivot.setPosition(subsystems.pivot(), ()-> ReverseFeed).withTimeout(.5),
                DriveCommands.strafeLeft(subsystems.drive(),()-> Strafe2),
                DriveCommands.backward(subsystems.drive(),()->3),
                Claw.setPosition(subsystems.claw(),()-> ClawConstants.CLOSE).withTimeout(.4),
                (Pivot.setPosition(subsystems.pivot(), ()-> PivotConstants.HIGH)).withTimeout(.5),
                DriveCommands.backward(subsystems.drive(),()->BackDistance2),
                Pivot.setPosition(subsystems.pivot(),()->HIGH2).withTimeout(.2),
                Claw.setPosition(subsystems.claw(),()-> ClawConstants.OPEN).withTimeout(.5),
                DriveCommands.forward(subsystems.drive(),()->BackDistance2+3),
                Pivot.setPosition(subsystems.pivot(), ()-> ReverseFeed).withTimeout(.5),
                DriveCommands.strafeLeft(subsystems.drive(),()-> Strafe2),
                DriveCommands.backward(subsystems.drive(),()->4),
                Claw.setPosition(subsystems.claw(),()-> ClawConstants.CLOSE).withTimeout(.4),
                (Pivot.setPosition(subsystems.pivot(), ()-> PivotConstants.HIGH)).withTimeout(.5),
                DriveCommands.backward(subsystems.drive(),()->BackDistance2),
                Pivot.setPosition(subsystems.pivot(),()->HIGH2).withTimeout(.2),
                Claw.setPosition(subsystems.claw(),()-> ClawConstants.OPEN).withTimeout(.5),
                DriveCommands.forward(subsystems.drive(),()->BackDistance2+2),
                Pivot.setPosition(subsystems.pivot(),()->0.5).withTimeout(.3),
                DriveCommands.turn(subsystems.drive(),()->180),
                DriveCommands.strafeLeft(subsystems.drive(),()->40),
                DriveCommands.driveToPose(subsystems.drive(),()->new Pose(0, 0,Math.toRadians( 180)))
        );
    }

    public static Command redAuto(Subsystems subsystems) {
        return Commands.sequence(
                Commands.parallel(
                        Claw.setPosition(subsystems.claw(),()-> ClawConstants.OPEN).withTimeout(.1),
                        DriveCommands.driveToPose(subsystems.drive(),()->new Pose(-BackDistance, -Strafe1, 0)),
                        (Pivot.setPosition(subsystems.pivot(), ()-> ReverseFeed)),
                        (Wrist.setPosition(subsystems.wrist(), ()-> WristConstants.FLIPPED)).withTimeout(.1)
                ).withTimeout(1.7),
                Claw.setPosition(subsystems.claw(),()-> ClawConstants.CLOSE).withTimeout(.4),
                (Pivot.setPosition(subsystems.pivot(), ()-> PivotConstants.HIGH)).withTimeout(.5),
                DriveCommands.backward(subsystems.drive(),()->BackDistance2),
                Pivot.setPosition(subsystems.pivot(),()->HIGH2).withTimeout(.2),
                Claw.setPosition(subsystems.claw(),()-> ClawConstants.OPEN).withTimeout(.5),
                DriveCommands.forward(subsystems.drive(),()->BackDistance2+2),
                Pivot.setPosition(subsystems.pivot(), ()-> ReverseFeed).withTimeout(.5),
                DriveCommands.strafeRight(subsystems.drive(),()-> Strafe2),
                DriveCommands.backward(subsystems.drive(),()->2),
                Claw.setPosition(subsystems.claw(),()-> ClawConstants.CLOSE).withTimeout(.4),
                (Pivot.setPosition(subsystems.pivot(), ()-> PivotConstants.HIGH)).withTimeout(.5),
                DriveCommands.backward(subsystems.drive(),()->BackDistance2+1),
                Pivot.setPosition(subsystems.pivot(),()->HIGH2).withTimeout(.2),
                Claw.setPosition(subsystems.claw(),()-> ClawConstants.OPEN).withTimeout(.5),
                DriveCommands.forward(subsystems.drive(),()->BackDistance2+3),
                Pivot.setPosition(subsystems.pivot(), ()-> ReverseFeed).withTimeout(.5),
                DriveCommands.strafeRight(subsystems.drive(),()-> Strafe2),
                DriveCommands.backward(subsystems.drive(),()->2),
                Claw.setPosition(subsystems.claw(),()-> ClawConstants.CLOSE).withTimeout(.4),
                (Pivot.setPosition(subsystems.pivot(), ()-> PivotConstants.HIGH)).withTimeout(.5),
                DriveCommands.backward(subsystems.drive(),()->BackDistance2),
                Pivot.setPosition(subsystems.pivot(),()->HIGH2).withTimeout(.2),
                Claw.setPosition(subsystems.claw(),()-> ClawConstants.OPEN).withTimeout(.5),
                DriveCommands.forward(subsystems.drive(),()->BackDistance2+2),
                Pivot.setPosition(subsystems.pivot(),()->0.5).withTimeout(.3),
                DriveCommands.turn(subsystems.drive(),()->180),
                DriveCommands.driveToPose(subsystems.drive(),()->new Pose(0, 0,Math.toRadians( 180)))
                );

    }
}