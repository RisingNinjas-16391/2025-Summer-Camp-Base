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
import org.firstinspires.ftc.teamcode.subsystems.climber_pivot.ClimberPivot;
import org.firstinspires.ftc.teamcode.subsystems.climber_pivot.ClimberPivotConstants;
import org.firstinspires.ftc.teamcode.subsystems.drive.Drive;
import org.firstinspires.ftc.teamcode.subsystems.pivot.Pivot;
import org.firstinspires.ftc.teamcode.subsystems.pivot.PivotConstants;
import org.firstinspires.ftc.teamcode.subsystems.wrist.Wrist;
import org.firstinspires.ftc.teamcode.subsystems.wrist.WristConstants;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class RobotContainer {
    private final Drive drive;
    private final Pivot pivot;
    private final Claw claw;
    private final Wrist wrist;
    private final ClimberPivot climb;

    private final Subsystems subsystems;

    private final CommandGamepad driverController;

    public RobotContainer(HardwareMap hwMap, Telemetry telemetry, Gamepad gamepad1, Gamepad gamepad2, int autoNum) {
        drive = new Drive(hwMap, telemetry);
        pivot = new Pivot(hwMap, telemetry);
        claw = new Claw(hwMap, telemetry);
        wrist = new Wrist(hwMap, telemetry);
        climb = new ClimberPivot(hwMap, telemetry);

        subsystems = new Subsystems(drive, pivot, claw, wrist, climb);

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
    }

    public void configureButtonBindings() {
        driverController.a().onTrue(Pivot.setPosition(pivot, () -> PivotConstants.FEED)
                .alongWith(Wrist.setPosition(wrist, () -> WristConstants.FLIPPED))
                .alongWith(ClimberPivot.setPosition(climb, () -> ClimberPivotConstants.TRAVEL)));
        driverController.b().onTrue(Pivot.setPosition(pivot, () -> PivotConstants.LOW)
                .alongWith(Wrist.setPosition(wrist, () -> WristConstants.UPRIGHT))
                .alongWith(ClimberPivot.setPosition(climb, () -> ClimberPivotConstants.TRAVEL)));
        driverController.y().onTrue(Pivot.setPosition(pivot, () -> PivotConstants.HIGH)
                .alongWith(Wrist.setPosition(wrist, () -> WristConstants.UPRIGHT))
                .alongWith(ClimberPivot.setPosition(climb, () -> ClimberPivotConstants.TRAVEL)));
        driverController.x().onTrue(
                Commands.parallel(
                        Pivot.setPosition(pivot, () -> PivotConstants.FEED),
                        ClimberPivot.setPosition(climb, () -> ClimberPivotConstants.CLIMB)));

        driverController.rightBumper().onTrue(Pivot.score(pivot).andThen(Claw.setPosition(claw, () -> ClawConstants.OPEN)));

        driverController.leftTrigger().onTrue(Claw.setPosition(claw, () -> ClawConstants.OPEN));
        driverController.rightTrigger().onTrue(Claw.setPosition(claw, () -> ClawConstants.CLOSE));
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