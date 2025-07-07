package org.firstinspires.ftc.teamcode.commands.auto;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.localization.Pose;

@Config
public class TunablePose {
    public double x;
    public double y;
    public double heading;

    public TunablePose(double x, double y, double heading) {
        this.x = x;
        this.y = y;
        this.heading = heading;
    }

    public Pose getPose() {
        return new Pose(x, y, Math.toRadians(heading));
    }
}
