package org.firstinspires.ftc.teamcode.subsystems.pivot;

import com.acmerobotics.dashboard.config.Config;

@Config
public class PivotConstants {
    public static double kP = 3;
    public static double kG = 0.04;

    public static double ticksToRotations = 1.0 / (751.8 * 5);

    public static double setpoint = -0.12;

    public static double initialPosition = 0;

    public static double FEED = 0.035;
    public static double CONE = -0.775;
    public static double HIGH = 0.3;
    public static double HIGH2 = -0.71;

    public static double currentThreshold = 2;
    public static double resetVoltage = 10;

}
