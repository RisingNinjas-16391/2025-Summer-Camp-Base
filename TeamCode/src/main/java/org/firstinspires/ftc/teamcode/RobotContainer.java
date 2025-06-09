package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.commands.AutoCommands;
import org.firstinspires.ftc.teamcode.commands.DriveCommands;
import org.firstinspires.ftc.teamcode.lib.wpilib.CommandGamepad;
import org.firstinspires.ftc.teamcode.subsystems.Subsystems;
import org.firstinspires.ftc.teamcode.subsystems.claw.Claw;
import org.firstinspires.ftc.teamcode.subsystems.claw.ClawConstants;
import org.firstinspires.ftc.teamcode.subsystems.drive.Drive;
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake;
import org.firstinspires.ftc.teamcode.subsystems.pivot.Pivot;
import org.firstinspires.ftc.teamcode.subsystems.pivot.PivotConstants;
import org.firstinspires.ftc.teamcode.subsystems.servo_intake.ServoIntake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class RobotContainer {
    private final Drive drive;
    private final Pivot pivot;
//    private final Claw claw;
    private final ServoIntake servoIntake;

//    private final Intake intake;
//    private final Intake shooter;

    private final Subsystems subsystems;

    private final CommandGamepad driverController;

    public RobotContainer(HardwareMap hwMap, Telemetry telemetry, Gamepad gamepad1, Gamepad gamepad2, int autoNum) {
        drive = new Drive(hwMap, telemetry);
        pivot = new Pivot(hwMap, telemetry);
//        claw = new Claw(hwMap, telemetry);
        servoIntake = new ServoIntake(hwMap, telemetry);
//        intake = new Intake(hwMap, telemetry, "intake");
//        shooter = new Intake(hwMap, telemetry, "shooter");

        subsystems = new Subsystems(drive, pivot, servoIntake);

        driverController = new CommandGamepad(gamepad1);

        if (autoNum == 0) {
            setDefaultCommands();
            configureButtonBindings();
        } else {
            getAutoCommand(autoNum);
        }
    }

    public void setDefaultCommands(){
        drive.setDefaultCommand(
                DriveCommands.joystickDrive(
                        drive,
                        () -> DriveCommands.signSquare(-driverController.getLeftY()),
                        () -> DriveCommands.signSquare(-driverController.getLeftX()),
                        () -> DriveCommands.signSquare(-driverController.getRightX()))
        );

        servoIntake.setDefaultCommand(ServoIntake.setPower(servoIntake, () -> driverController.getRightTrigger() - driverController.getLeftTrigger()));
//        intake.setDefaultCommand(Intake.setPower(intake, () -> driverController.getLeftTrigger() - driverController.getRightTrigger()));
    }

    public void configureButtonBindings() {
//        driverController.a().whileTrue(Intake.setPower(shooter, () -> 0.35)).onFalse(Intake.setPower(shooter, () -> 0.0));
//        driverController.b().whileTrue(Intake.setPower(shooter, () -> -0.35)).onFalse(Intake.setPower(shooter, () -> 0.0));

        driverController.a().onTrue(Pivot.setPosition(pivot, () -> PivotConstants.LOW));
        driverController.b().onTrue(Pivot.setPosition(pivot, () -> PivotConstants.MIDDLE));
        driverController.y().onTrue(Pivot.setPosition(pivot, () -> PivotConstants.HIGH));

//        driverController.leftTrigger().onTrue(Claw.setPosition(claw, () -> ClawConstants.OPEN));
//        driverController.rightTrigger().onTrue(Claw.setPosition(claw, () -> ClawConstants.CLOSE));
    }

    public Command getAutoCommand(int chooser) {
        switch (chooser) {
            case 1:
                return AutoCommands.blueAuto(subsystems);
            case 2:
                return AutoCommands.redAuto(subsystems);
        }
        return Commands.none();
    }
}