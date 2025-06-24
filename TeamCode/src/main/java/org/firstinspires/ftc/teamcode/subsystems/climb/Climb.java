package org.firstinspires.ftc.teamcode.subsystems.climb;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.lib.ftclib.hardware.ServoEx;
import org.firstinspires.ftc.teamcode.lib.ftclib.hardware.motors.CRServo;
import org.firstinspires.ftc.teamcode.subsystems.servo_intake.ServoIntakeConstants;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climb extends SubsystemBase {
    private final Telemetry telemetry;

    private final CRServo climb;

    private double kSetpoint = ClimbConstants.setpoint;

    public Climb(HardwareMap hwMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        climb = new CRServo(hwMap, "climb");

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
        climb.set(power);
    }

    public static Command setPower(Climb climb, DoubleSupplier power) {
        return Commands.run(() -> climb.setPower(power.getAsDouble()), climb);
    }
}
