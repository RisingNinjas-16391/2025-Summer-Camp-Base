package org.firstinspires.ftc.teamcode.commands;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.localization.Pose;

@Config
public class TunablePose {
    public double x;
    public double y;
    public double heading;

    public TunablePose(double x, double y, double headingDegrees) {
        this.x = x;
        this.y = y;
        this.heading = headingDegrees;
    }

    public Pose getPose() {
        return new Pose(x, y, Math.toRadians(heading));
    }
}
