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
import org.firstinspires.ftc.teamcode.subsystems.climb.Climb;
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
    private final Wrist wrist;
    private final Claw claw;
    private final Climb climb;

    private final Subsystems subsystems;

    private final CommandGamepad driverController;

    public RobotContainer(HardwareMap hwMap, Telemetry telemetry, Gamepad gamepad1, Gamepad gamepad2, int autoNum) {
        drive = new Drive(hwMap, telemetry);
        pivot = new Pivot(hwMap, telemetry);
        wrist = new Wrist(hwMap, telemetry);
        claw = new Claw(hwMap, telemetry);
        climb = new Climb(hwMap, telemetry);

        subsystems = new Subsystems(drive, pivot, wrist, claw, climb);

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
        driverController.a().onTrue(Pivot.setPosition(pivot, () -> PivotConstants.FEED).alongWith(Wrist.setPosition(wrist, () -> WristConstants.UPRIGHT)));
        driverController.b().onTrue(Pivot.setPosition(pivot, () -> PivotConstants.LOW).alongWith(Wrist.setPosition(wrist, () -> WristConstants.FLIPPED)));
        driverController.y().onTrue(Pivot.setPosition(pivot, () -> PivotConstants.HIGH).alongWith(Wrist.setPosition(wrist, () -> WristConstants.FLIPPED)));
        driverController.x().onTrue(Pivot.setPosition(pivot, () -> PivotConstants.CLIMB).alongWith(Wrist.setPosition(wrist, () -> WristConstants.FLIPPED)));

        driverController.rightBumper().onTrue(Pivot.score(pivot).andThen(Claw.setPosition(claw, () -> ClawConstants.OPEN)));

        driverController.leftBumper().onTrue(Climb.setPower(climb, () -> 1.0)).onFalse(Climb.setPower(climb, () -> 0.0));

        driverController.dpadUp().onTrue(Climb.setPower(climb, () -> -1.0));

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