package org.firstinspires.ftc.teamcode.subsystems;

import org.firstinspires.ftc.teamcode.subsystems.claw.Claw;
import org.firstinspires.ftc.teamcode.subsystems.climber_pivot.ClimberPivot;
import org.firstinspires.ftc.teamcode.subsystems.drive.Drive;
import org.firstinspires.ftc.teamcode.subsystems.pivot.Pivot;
import org.firstinspires.ftc.teamcode.subsystems.servo_intake.ServoIntake;
import org.firstinspires.ftc.teamcode.subsystems.wrist.Wrist;

//public record Subsystems(Drive drive, Pivot pivot, Claw claw) {
//}

public record Subsystems(Drive drive, Pivot pivot, Claw claw, Wrist wrist, ClimberPivot climb) {
}
