package org.firstinspires.ftc.teamcode.subsystems.climber_pivot;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.lib.controller.SquIDController;
import org.firstinspires.ftc.teamcode.lib.ftclib.hardware.motors.MotorEx;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimberPivot extends SubsystemBase {
    private final Telemetry telemetry;

    private final MotorEx ClimberPivotMotor;

    private final SquIDController controller;

    private double kSetpoint;

    private double currentPosition = ClimberPivotConstants.initialPosition;
    private double desiredPosition = ClimberPivotConstants.initialPosition;

    public ClimberPivot(HardwareMap hwMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        ClimberPivotMotor = new MotorEx(hwMap, "climberPivot");

        ClimberPivotMotor.setInverted(false);

        ClimberPivotMotor.stopAndResetEncoder();

        controller = new SquIDController(0.1);

        kSetpoint = ClimberPivotConstants.setpoint;
    }

    @Override
    public void periodic() {
        try {
            currentPosition = ClimberPivotMotor.getCurrentPosition() * ClimberPivotConstants.ticksToRotations + ClimberPivotConstants.initialPosition;

            telemetry.addLine("ClimberPivot");
            telemetry.addData("ClimberPivot Position", currentPosition);
            telemetry.addData("ClimberPivot Voltage", ClimberPivotMotor.getVoltage());
            telemetry.addData("ClimberPivot Current", ClimberPivotMotor.getCurrent());

            // If setpoint on dashboard changes, update the setpoint
            if (kSetpoint != ClimberPivotConstants.setpoint) {
                kSetpoint = ClimberPivotConstants.setpoint;
                desiredPosition = kSetpoint;
            }

            calculateVoltage(desiredPosition);
        } catch (Exception ignored) {

        }
    }

    private void calculateVoltage(double position) {
        telemetry.addData("ClimberPivot Desired Position", position);

        double output = controller.calculate(ClimberPivotConstants.kP, position, currentPosition) + ClimberPivotConstants.kG * Math.cos(Units.rotationsToRadians(currentPosition));
        ClimberPivotMotor.set(output);
    }

    private void setPosition(double position) {
        desiredPosition = position;
    }

    public double getPosition() {
        return currentPosition;
    }

    public boolean isFinished() {
        return Math.abs(currentPosition - desiredPosition) < 0.05;
    }
    public static Command setPosition(ClimberPivot ClimberPivot, DoubleSupplier position) {
        return Commands.run(() -> ClimberPivot.setPosition(position.getAsDouble()), ClimberPivot).until(ClimberPivot::isFinished);
    }
}
