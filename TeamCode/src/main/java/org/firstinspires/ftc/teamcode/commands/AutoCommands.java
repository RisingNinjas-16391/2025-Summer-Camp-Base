package org.firstinspires.ftc.teamcode.commands;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.localization.Pose;

import org.firstinspires.ftc.teamcode.subsystems.Subsystems;
import org.firstinspires.ftc.teamcode.subsystems.claw.Claw;
import org.firstinspires.ftc.teamcode.subsystems.drive.Drive;
import org.firstinspires.ftc.teamcode.subsystems.pivot.Pivot;
import org.firstinspires.ftc.teamcode.subsystems.pivot.PivotConstants;

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
    public static Command blueAuto(Subsystems subsystems) {
        return Commands.sequence(
                Claw.close(subsystems.claw()).withTimeout(1.0),
                //close at begging
                DriveCommands.forward(subsystems.drive(), 15),
                DriveCommands.strafeRight(subsystems.drive(), 15),
                Pivot.setPosition(subsystems.pivot(), PivotConstants.LOWBARHIGH).withTimeout(0.0),
                DriveCommands.forward(subsystems.drive(), 25),
                DriveCommands.turn(subsystems.drive(), 90).withTimeout(2.0),
                DriveCommands.forward(subsystems.drive(), 5),
                Pivot.setPosition(subsystems.pivot(), PivotConstants.FEED).withTimeout(0.0),
                Pivot.setPosition(subsystems.pivot(), PivotConstants.DOWN).withTimeout(0.0),
                Claw.open(subsystems.claw()).withTimeout(1.0),
                DriveCommands.backward(subsystems.drive(), 15),
                DriveCommands.turn(subsystems.drive(), -270).withTimeout(1.5),
                //go to second
                DriveCommands.strafeLeft(subsystems.drive(), 5),
                DriveCommands.forward(subsystems.drive(), 30),
                Claw.close(subsystems.claw()).withTimeout(1.0),
                DriveCommands.backward(subsystems.drive(), 24),
                Pivot.setPosition(subsystems.pivot(), PivotConstants.LOWBARHIGH).withTimeout(0.0),
                DriveCommands.turn(subsystems.drive(), -90).withTimeout(1.5),
                DriveCommands.forward(subsystems.drive(), 25),
                Claw.open(subsystems.claw()).withTimeout(0.0),
                Pivot.setPosition(subsystems.pivot(), PivotConstants.initialPosition).withTimeout(2.0),

                DriveCommands.backward(subsystems.drive(), 10),
                DriveCommands.driveToPose(subsystems.drive(),Pose::new)




                );
    }




    public static Command redAuto(Subsystems subsystems) {
        return Commands.sequence(
                Claw.close(subsystems.claw()).withTimeout(1.0),
                //close at begging
                DriveCommands.forward(subsystems.drive(), 15),
                DriveCommands.strafeLeft(subsystems.drive(), 15),
                Pivot.setPosition(subsystems.pivot(), PivotConstants.LOWBARHIGH).withTimeout(0.0),
                DriveCommands.forward(subsystems.drive(), 25),
                DriveCommands.turn(subsystems.drive(), -90).withTimeout(2.0),
                DriveCommands.forward(subsystems.drive(), 5),
                Pivot.setPosition(subsystems.pivot(), PivotConstants.FEED).withTimeout(0.0),
                Pivot.setPosition(subsystems.pivot(), PivotConstants.DOWN).withTimeout(0.0),
                Claw.open(subsystems.claw()).withTimeout(1.0),
                DriveCommands.backward(subsystems.drive(), 15),
                DriveCommands.turn(subsystems.drive(), 270).withTimeout(1.5),
                //go to second
                DriveCommands.strafeRight(subsystems.drive(), 6),
                DriveCommands.forward(subsystems.drive(), 35),
                Claw.close(subsystems.claw()).withTimeout(1.0),
                DriveCommands.backward(subsystems.drive(), 25),
                Pivot.setPosition(subsystems.pivot(), PivotConstants.LOWBARHIGH).withTimeout(0.0),
                DriveCommands.turn(subsystems.drive(), 90).withTimeout(1.5),
                DriveCommands.forward(subsystems.drive(), 25),
                Claw.open(subsystems.claw()).withTimeout(0.0),
                Pivot.setPosition(subsystems.pivot(), PivotConstants.initialPosition).withTimeout(2.0),

                DriveCommands.backward(subsystems.drive(), 10),
                DriveCommands.driveToPose(subsystems.drive(),Pose::new)

















                //to do score second


        );
    }
}