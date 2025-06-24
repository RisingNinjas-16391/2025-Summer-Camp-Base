package org.firstinspires.ftc.teamcode.subsystems.pivot;

import com.acmerobotics.dashboard.config.Config;

@Config
public class PivotConstants {
    public static double kP = 3;
    public static double kG = 0.04;

    public static double ticksToRotations = 1.0 / (751.8 * 5);

    public static double setpoint = 0.5;

    public static double initialPosition = 0.5;

    public static double LOW = 0.15;
    public static double MIDDLE = 0.62;
    public static double HIGH = 0.55;
    public static double CLIMBSET = 0.75;
    public static double CLIMB = 0.6;
    public static double SCORE =0.7;
}
