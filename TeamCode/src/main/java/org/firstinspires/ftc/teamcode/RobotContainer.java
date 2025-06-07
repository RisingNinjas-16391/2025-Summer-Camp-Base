package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.commands.AutoCommands;
import org.firstinspires.ftc.teamcode.commands.DriveCommands;
import org.firstinspires.ftc.teamcode.lib.wpilib.CommandGamepad;
import org.firstinspires.ftc.teamcode.subsystems.drive.Drive;
//import org.firstinspires.ftc.teamcode.subsystems.intake.Intake;
import org.firstinspires.ftc.teamcode.subsystems.servo_intake.ServoIntake;
import org.firstinspires.ftc.teamcode.subsystems.pivot.Pivot;


import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class RobotContainer {
    private final Drive drive;
    private final Pivot pivot;

    private final ServoIntake leftServo;
    private final ServoIntake rightServo;
    //private final Intake intake;
    //private final Intake shooter;

    private final CommandGamepad driverController;

    public RobotContainer(HardwareMap hwMap, Telemetry telemetry, Gamepad gamepad1, Gamepad gamepad2, int autoNum) {
        drive = new Drive(hwMap, telemetry);
        //pivot = new Pivot(hwMap, telemetry);
        leftServo = new ServoIntake(hwMap, telemetry);
        rightServo = new ServoIntake(hwMap, telemetry);
        intake = new Intake(hwMap, telemetry, "intake");
        //shooter = new Intake(hwMap, telemetry, "shooter");

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
                        () -> -driverController.getLeftY(),
                        () -> -driverController.getLeftX(),
                        () -> -driverController.getRightX()));

        //leftServo.setDefaultCommand(ServoIntake.setLeftServoPower(leftServo, driverController::getLeftTrigger));
        //rightServo.setDefaultCommand(ServoIntake.setRightServoPower(rightServo, driverController::getRightTrigger));

    }

    public void configureButtonBindings() {
        //driverController.a().whileTrue(Intake.setPower(shooter, () -> 0.35)).onFalse(Intake.setPower(shooter, () -> 0.0));
        //driverController.b().whileTrue(Intake.setPower(shooter, () -> -0.35)).onFalse(Intake.setPower(shooter, () -> 0.0));
        //driverController.y().whileTrue(Pivot.setPosition(pivot, () -> 0.35)).onFalse(Pivot.setPosition(pivot, () -> 0.0));
        //driverController.x().onTrue(Pivot.setPosition(pivot, () -> -1));
        //driverController.y().onTrue(Pivot.setPosition(pivot, () -> 0.25));
        driverController.a().onTrue(ServoIntake.setLeftServoPower(leftServo, () -> 0.5)).onFalse(ServoIntake.setLeftServoPower(leftServo, () -> -0.5));
        driverController.a().onTrue(ServoIntake.setRightServoPower(rightServo, () -> -0.5)).onFalse(ServoIntake.setRightServoPower(rightServo, () -> 0.5));

    }

    public Command getAutoCommand(int chooser) {
        switch (chooser) {
            case 1:
//            return AutoCommands.blueAuto(drive, pivot);
        }
        return Commands.none();
    }
}