package org.firstinspires.ftc.teamcode.commands;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.localization.Pose;

import org.firstinspires.ftc.teamcode.subsystems.Subsystems;
import org.firstinspires.ftc.teamcode.subsystems.drive.Drive;
import org.firstinspires.ftc.teamcode.subsystems.pivot.Pivot;
import org.firstinspires.ftc.teamcode.subsystems.pivot.PivotConstants;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

@Config
public class AutoCommands {

    public static Command blueAuto(Subsystems subsystems) {
        return Commands.sequence(
                DriveCommands.driveToPose(
                        subsystems.drive(),
                        () -> new Pose(10, 10, 0)),
                DriveCommands.driveToPose(
                        subsystems.drive(),
                        () -> new Pose(0, 20, 0))
        );
    }

    public static Command redAuto(Subsystems subsystems) {
        return Commands.sequence(

        );
    }
}