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

    DriveCommands.driveToPose(subsystems.drive(), () -> new Pose(x, y, Math.toRadians(heading)))


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
                    Shooter.setPower(subsystems.shooter(), 1).withTimeout(2),


    */
    public static Command blueAuto(Subsystems subsystems) {
        return Commands.sequence(
                DriveCommands.driveToPose(subsystems.drive(), () -> new Pose(60, -34, Math.toRadians(135))),
                Shooter.setPower(subsystems.shooter(), -0.7).withTimeout(2),
                Intake.setPower(subsystems.intake(), -1).withTimeout(1),
                Shooter.setPower(subsystems.shooter(), 0).withTimeout(4),
                DriveCommands.driveToPose(subsystems.drive(), () -> new Pose(65, -12.5, Math.toRadians(45))),
                Intake.setPower(subsystems.intake(), -1).withTimeout(0.5),
                Shooter.setPower(subsystems.shooter(), 0.1).withTimeout(0.2),
                Intake.setPower(subsystems.intake(), 0.1).withTimeout(0.2),
                DriveCommands.driveToPose(subsystems.drive(), () -> new Pose(60, -34, Math.toRadians(135))),
                Shooter.setPower(subsystems.shooter(), -0.7).withTimeout(2),
                Intake.setPower(subsystems.intake(), -1).withTimeout(1),
                Shooter.setPower(subsystems.shooter(), 0).withTimeout(4)








                /*list of things to do (also kinda a command list ig):
                    Provided that you're facing towards the wall with the BLUE pipe!
                move towards the middle of the field (___ inches for distance)
                    Prelim measurement was something like 43 inches, but this was from red's position
                turn to the left (___ degrees)
                    should just be 90 degrees...
                move forward towards the hoops (___ inches for distance)
                turn to the left a bit more, aiming for the tee (___ degrees)
                    the preset ball, located in the top left of the field (from birds' eye view)
                    rough estimate for this measurement would be roughly 45-ish degrees ig
                grab the ball (___ inches distance)
                return back to previous spot (___ inches)
                    same command as the movement directly before this, just moving backwards
                turn to face hoop (___ degrees)
                    perhaps 90 degrees
                shoot the ball (___ rpm?? dunno but you gotta use the flywheels)
                    make sure to not go above 9-ish volts for the input of the top motor (the one attached to the orange wheels)
                    Javier says that going above this will cause the control hub to shut down the motor for safety purposes.
                    He also says that we'll figure out the right amount
                */

        );
    }

    public static Command redAuto(Subsystems subsystems) {
        return Commands.sequence(
                DriveCommands.driveToPose(subsystems.drive(), () -> new Pose(60, 34, Math.toRadians(-135))),
                Shooter.setPower(subsystems.shooter(), -0.75).withTimeout(2),
                Intake.setPower(subsystems.intake(), -0.75).withTimeout(1),
                Shooter.setPower(subsystems.shooter(), 0).withTimeout(2.5),
                DriveCommands.driveToPose(subsystems.drive(), () -> new Pose(67, 12.5, Math.toRadians(-45))),
                Intake.setPower(subsystems.intake(), -1).withTimeout(0.5),
                Shooter.setPower(subsystems.shooter(), 0.1).withTimeout(0),
                Intake.setPower(subsystems.intake(), 0.1).withTimeout(0),
                DriveCommands.driveToPose(subsystems.drive(), () -> new Pose(60, 34, Math.toRadians(-135))),
                Shooter.setPower(subsystems.shooter(), -0.75).withTimeout(2),
                Intake.setPower(subsystems.intake(), -0.75).withTimeout(1),
                Shooter.setPower(subsystems.shooter(), 0).withTimeout(2.5),
                DriveCommands.driveToPose(subsystems.drive(), () -> new Pose(0, 0, Math.toRadians(0)))





                /*
                list of things:
                - move to position near hoop
                    position might be the same as blue, but reflected over a y-axis within the field
                - shoot preset ball
                    should just use the same power as the blue side
                - go to second preset ball
                    once again, same position reflected?
                - load 2nd ball

                - move to position near hoop again
                - shoot 2nd ball
                 */

                );
    }
}


/* ideas for teleop:
left stick: movement

right stick: turning robot
right trigger (RT): launch ball
either x button or left trigger: feeding (reloading)
 */