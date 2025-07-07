package org.firstinspires.ftc.teamcode.commands;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.localization.Pose;

import org.firstinspires.ftc.teamcode.subsystems.Subsystems;
import org.firstinspires.ftc.teamcode.subsystems.drive.Drive;
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake;
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
    DriveCommands.strafeRight(subsystems.drive(), distance),
    DriveCommands.strafeLeft(subsystems.drive(), distance),



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
                DriveCommands.setPose(subsystems.drive(), () -> new Pose(0, 0, Math.toRadians(-90))),
                //begin
                Pivot.setPosition(subsystems.pivot(), () -> PivotConstants.FEED),
                DriveCommands.forward(subsystems.drive(), 55),
                DriveCommands.strafeLeft(subsystems.drive(), 55),
                //lift arm up
                Pivot.setPosition(subsystems.pivot(), () -> PivotConstants.HIGH),
                DriveCommands.forward(subsystems.drive(), 15.5),
                //toss out is negative
                //toss out

                Intake.setPower(subsystems.intake(), 1).withTimeout(1),
                //wait 1 second then stop
                Intake.setPower(subsystems.intake(), 0).withTimeout(0.0),
                //move backwards
                DriveCommands.backward(subsystems.drive(), 10),
                //afterwards pivot goes back down
                Pivot.setPosition(subsystems.pivot(), () -> PivotConstants.FEED),

                //goes to second ball
                DriveCommands.strafeRight(subsystems.drive(), 50),
                //grab
                //takes in then stops
                Intake.setPower(subsystems.intake(), -1).withTimeout(1),
                DriveCommands.forward(subsystems.drive(), 8.5),
                Intake.setPower(subsystems.intake(), 0).withTimeout(0.0),
                DriveCommands.backward(subsystems.drive(), 15),
                //goes to toss
                DriveCommands.strafeLeft(subsystems.drive(), 55),
                Pivot.setPosition(subsystems.pivot(), () -> PivotConstants.HIGH),
                DriveCommands.forward(subsystems.drive(), 15),

                //toss

                Intake.setPower(subsystems.intake(), 1).withTimeout(1),
                Intake.setPower(subsystems.intake(), 0).withTimeout(0.0),
                DriveCommands.backward(subsystems.drive(), 70),
                //parks

                DriveCommands.strafeRight(subsystems.drive(), 60)

        );
    }

    public static Command redAuto(Subsystems subsystems) {
        return Commands.sequence(
                DriveCommands.setPose(subsystems.drive(), () -> new Pose(0, 0, Math.toRadians(90))),
                //begin
                Pivot.setPosition(subsystems.pivot(), () -> PivotConstants.FEED),
                DriveCommands.forward(subsystems.drive(), 55),
                DriveCommands.strafeRight(subsystems.drive(), 50),
                //lift arm up
                Pivot.setPosition(subsystems.pivot(), () -> PivotConstants.HIGH),
                DriveCommands.forward(subsystems.drive(), 15.5),
                //toss out is negative
                //toss out

                Intake.setPower(subsystems.intake(), 1).withTimeout(1),
                //wait 1 second then stop
                Intake.setPower(subsystems.intake(), 0).withTimeout(0.0),
                //move backwards
                DriveCommands.backward(subsystems.drive(), 10),
                //afterwards pivot goes back down
                Pivot.setPosition(subsystems.pivot(), () -> PivotConstants.FEED),

                //goes to second ball
                DriveCommands.strafeLeft(subsystems.drive(), 52),
                //grab
                //takes in then stops
                Intake.setPower(subsystems.intake(), -1).withTimeout(1),
                DriveCommands.forward(subsystems.drive(), 8),
                Intake.setPower(subsystems.intake(), 0).withTimeout(0.0),
                DriveCommands.backward(subsystems.drive(), 15),
                //goes to toss
                DriveCommands.strafeRight(subsystems.drive(), 49),
                Pivot.setPosition(subsystems.pivot(), () -> PivotConstants.HIGH),
                DriveCommands.forward(subsystems.drive(), 15),

                //toss

                Intake.setPower(subsystems.intake(), 1).withTimeout(1),
                Intake.setPower(subsystems.intake(), 0).withTimeout(0.0),
                DriveCommands.backward(subsystems.drive(), 70),
                //parks

                DriveCommands.strafeLeft(subsystems.drive(), 60)

        );
    }
}