package com.swrobotics.robot.logging;

import com.swrobotics.robot.control.AimCalc;

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public final class Telemetry {
    private static final Mechanism2d mech = new Mechanism2d(3.0, 3.0);
    private static final MechanismRoot2d root = mech.getRoot("shooter", 1.5, 0.2);
    private static final MechanismLigament2d hoodLig =
        root.append(new MechanismLigament2d("hood", 1.0, 0.0)); // length 1, angle 0

    private Telemetry() {}

    public static void init() {
        SmartDashboard.putData("ShooterMech2d", mech);
    }

    public static void periodic() {
        AimCalc aim = AimCalc.getInstance();

        double distance = aim.getLastDistanceToHub();
        double hoodDeg = aim.getHoodAngle().getDegrees();
        double targetRps = aim.getShooterRPS();

        // Update mech graphic
        hoodLig.setAngle(hoodDeg);

        // Scalars for logging/analysis
        SmartDashboard.putNumber("Shooter/DistanceToHub", distance);
        SmartDashboard.putNumber("Shooter/HoodAngleDeg", hoodDeg);
        SmartDashboard.putNumber("Shooter/TargetRPS", targetRps);
    }
}