package org.firstinspires.ftc.teamcode.subsystems.climber_pivot;

import com.acmerobotics.dashboard.config.Config;

@Config
public class ClimberPivotConstants {
    public static double kP = 0.05;
    public static double kG = 0;

    public static double ticksToRotations = 1.0;

    public static double setpoint = 0.0;

    public static double initialPosition = 0.0;

    public static double PREPARE_CLIMB = 0;
    public static double CLIMB = -1250;
    public static double TRAVEL = 500;
}
