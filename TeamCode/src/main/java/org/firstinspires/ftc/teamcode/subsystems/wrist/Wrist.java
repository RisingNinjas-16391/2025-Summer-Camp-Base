package org.firstinspires.ftc.teamcode.subsystems.wrist;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Wrist extends SubsystemBase {
    private final Telemetry telemetry;

    private final Servo wrist;

    private double kSetpoint = WristConstants.setpoint;

    public Wrist(HardwareMap hwMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        wrist = hwMap.get(Servo.class, "wrist");

    }

    @Override
    public void periodic() {
        try {
            if (kSetpoint != WristConstants.setpoint) {
                kSetpoint = WristConstants.setpoint;
                setPosition(kSetpoint);
            }
        } catch (Exception ignored) {

        }
    }

    private void setPosition(double position) {
        wrist.setPosition(position);
    }

    public static Command setPosition(Wrist claw, DoubleSupplier power) {
        return Commands.run(() -> claw.setPosition(power.getAsDouble()), claw);
    }
}
