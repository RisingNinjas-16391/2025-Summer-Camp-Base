package org.firstinspires.ftc.teamcode.subsystems.pivot;

import com.acmerobotics.dashboard.config.Config;

@Config
public class PivotConstants {
    public static double kP = 3;
    public static double kG = 0.04;

    public static double ticksToRotations = 1.0 / (751.8 * 5);

    public static double setpoint = 0.5;

    public static double initialPosition = 0.5;

    public static double FEED = 0.855;
    public static double LOW = 0.36;
    public static double HIGH = 0.43;
    ;
    public static double HIGH_SAME_SIDE = 0.57;
    public static double FEEDSAME = 0.83;
    public static double CLIMB = .855;
    public static double SCORE = 0.61;
}
