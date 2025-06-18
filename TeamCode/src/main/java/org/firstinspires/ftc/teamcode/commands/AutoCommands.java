package org.firstinspires.ftc.teamcode.commands;

import android.os.DropBoxManager;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.localization.Pose;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.subsystems.Subsystems;
import org.firstinspires.ftc.teamcode.subsystems.drive.Drive;
import org.firstinspires.ftc.teamcode.subsystems.pivot.Pivot;
import org.firstinspires.ftc.teamcode.subsystems.pivot.PivotConstants;
import org.firstinspires.ftc.teamcode.subsystems.servo_intake.ServoIntake;
import org.firstinspires.ftc.teamcode.subsystems.servo_intake.ServoIntakeConstants;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

@Config
public class AutoCommands {

    public static Command redAuto(Subsystems subsystems) {
        return Commands.sequence(
                Pivot.setPosition(subsystems.pivot(), () -> PivotConstants.HILOW).withTimeout(0.1),
                DriveCommands.forward(subsystems.drive(), () -> 30.0),
                DriveCommands.strafeLeft(subsystems.drive(), () -> 12).withTimeout(0.5),
                ServoIntake.setPower(subsystems.servoIntake(), () -> 1.0).withTimeout(0.1),
                DriveCommands.forward(subsystems.drive(), () -> 5),
                Commands.waitSeconds(1),
                Pivot.setPosition(subsystems.pivot(), () -> PivotConstants.HIGH).withTimeout(0.4),
                DriveCommands.forward(subsystems.drive(), () -> 5),
                ServoIntake.setPower(subsystems.servoIntake(), () -> -1.0).withTimeout(0.1),
                Pivot.setPosition(subsystems.pivot(),() -> PivotConstants.HILOW).withTimeout(0.8),
                DriveCommands.backward(subsystems.drive(), () -> 0),
                DriveCommands.strafeLeft(subsystems.drive(), () -> 15).withTimeout(0.5)

        );
    }

    public static Command blueAuto(Subsystems subsystems) {
        return Commands.sequence(DriveCommands.forward(subsystems.drive(), () -> 1));
    }
}