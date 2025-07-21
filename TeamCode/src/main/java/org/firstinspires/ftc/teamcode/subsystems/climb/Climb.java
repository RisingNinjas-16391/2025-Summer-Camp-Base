package org.firstinspires.ftc.teamcode.subsystems.climb;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.lib.ftclib.hardware.motors.CRServo;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climb extends SubsystemBase {
    private final Telemetry telemetry;

    private final CRServo leftServo;
    private final CRServo rightServo;

    private double kSetpoint = ClimbConstants.setpoint;

    public Climb(HardwareMap hwMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        leftServo = new CRServo(hwMap, "leftClimb");
        rightServo = new CRServo(hwMap, "rightClimb");

        leftServo.setInverted(true);
        rightServo.setInverted(false);
    }

    @Override
    public void periodic() {
        try {
            if (kSetpoint != ClimbConstants.setpoint) {
                kSetpoint = ClimbConstants.setpoint;
                setPower(kSetpoint);
            }
        } catch (Exception ignored) {

        }
    }

    private void setPower(double power) {
        leftServo.set(power);
        rightServo.set(power);
    }

    public static Command setPower(Climb climb, DoubleSupplier power) {
        return Commands.run(() -> climb.setPower(power.getAsDouble()), climb);
    }
}
