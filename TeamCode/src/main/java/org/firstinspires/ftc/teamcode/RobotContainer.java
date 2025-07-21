package org.firstinspires.ftc.teamcode;

import com.pedropathing.localization.Pose;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.commands.AutoCommands;
import org.firstinspires.ftc.teamcode.commands.DriveCommands;
import org.firstinspires.ftc.teamcode.commands.auto.PoseStorage;
import org.firstinspires.ftc.teamcode.lib.wpilib.CommandGamepad;
import org.firstinspires.ftc.teamcode.opmodes.OpModeConstants;
import org.firstinspires.ftc.teamcode.subsystems.Subsystems;
import org.firstinspires.ftc.teamcode.subsystems.climb.Climb;
import org.firstinspires.ftc.teamcode.subsystems.drive.Drive;
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake;
import org.firstinspires.ftc.teamcode.subsystems.intake.IntakeConstants;
import org.firstinspires.ftc.teamcode.subsystems.pivot.Pivot;
import org.firstinspires.ftc.teamcode.subsystems.pivot.PivotConstants;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class RobotContainer {
    private final Drive drive;
    private final Pivot pivot;
    private final Intake intake;
    private final Climb climb;

    private final Subsystems subsystems;

    private final CommandGamepad driverController;

    public RobotContainer(HardwareMap hwMap, Telemetry telemetry, Gamepad gamepad1, Gamepad gamepad2, OpModeConstants autoNum) {
        drive = new Drive(hwMap, telemetry);
        pivot = new Pivot(hwMap, telemetry);
        intake = new Intake(hwMap, telemetry);
        climb = new Climb(hwMap, telemetry);

        subsystems = new Subsystems(drive, pivot, intake, climb);

        driverController = new CommandGamepad(gamepad1);

        if (autoNum == OpModeConstants.TELEOP) {
            setDefaultCommands();
            configureButtonBindings();
        } else {
            getAutoCommand(autoNum);
        }
    }

    public void setDefaultCommands(){
        drive.setDefaultCommand(
                Commands.sequence(
                        DriveCommands.setPose(drive, () -> PoseStorage.currentPose),
                        DriveCommands.joystickDrive(
                                drive,
                                () -> DriveCommands.signSquare(-driverController.getLeftY()),
                                () -> DriveCommands.signSquare(-driverController.getLeftX()),
                                () -> DriveCommands.signSquare(-driverController.getRightX()))
                )

        );
    }

    public void configureButtonBindings() {
        driverController.a().onTrue(Pivot.setPosition(pivot, () -> PivotConstants.LOW_FEED));
        driverController.b().onTrue(Pivot.setPosition(pivot, () -> PivotConstants.HIGH_FEED));
        driverController.y().onTrue(Pivot.setPosition(pivot, () -> PivotConstants.HIGH_SCORE));
        driverController.x().onTrue(Pivot.setPosition(pivot, () -> PivotConstants.LOW_SCORE));

        driverController.dpadDown().onTrue(
                Commands.parallel(
                        Climb.setPower(climb, () -> -1.0),
                        Commands.waitSeconds(2.0).andThen(Pivot.setPosition(pivot, () -> PivotConstants.CLIMB_2))
                )
        ).onFalse(Climb.setPower(climb, () -> 0.0));

        driverController.dpadUp().onTrue(
                Commands.parallel(
                        Climb.setPower(climb, () -> 1.0),
                        Pivot.setPosition(pivot, () -> PivotConstants.CLIMB)
                )
        ).onFalse(Climb.setPower(climb, () -> 0.0));

        driverController.dpadRight().onTrue(
                Commands.sequence(
                        Pivot.score(pivot, 0.075),
                        Intake.setPower(intake, () -> IntakeConstants.OUTTAKE_POWER).withTimeout(0.5),
                        Intake.setPower(intake, () -> 0.0).withTimeout(0.5)
                )
        );

        driverController.dpadLeft().onTrue(
                Pivot.score(pivot, -0.01)
        );

        driverController.start().onTrue(Pivot.resetPosition(pivot));

//        driverController.rightBumper().onTrue(Pivot.score(pivot).andThen(Claw.setPosition(claw, () -> ClawConstants.OPEN)));
//
        driverController.leftTrigger().onTrue(Intake.setPower(intake, () -> IntakeConstants.OUTTAKE_POWER)).onFalse(Intake.setPower(intake, () -> 0.0));
        driverController.rightTrigger().onTrue(Intake.setPower(intake, () -> IntakeConstants.INTAKE_POWER)).onFalse(Intake.setPower(intake, () -> 0.0));
    }

    public Command getAutoCommand(OpModeConstants auto) {
        return switch (auto) {
            case BLUE_AUTO -> AutoCommands.blueAuto(subsystems);
            case RED_AUTO -> AutoCommands.redAuto(subsystems);
            default -> Commands.none();
        };
    }

    public Pose getDrivePose() {
        return drive.getPose();
    }
}