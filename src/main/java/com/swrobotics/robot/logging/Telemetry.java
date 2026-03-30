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
    // Distance label ligament — length represents distance visually
    private static final MechanismRoot2d distRoot = mech.getRoot("distLabel", 0.2, 2.5);
    private static final MechanismLigament2d distLig =
        distRoot.append(new MechanismLigament2d("distance", 0.0, 0.0));

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
        // Scale distance into the mech widget (0-5m maps to 0-2.5 length)
        distLig.setLength(Math.min(distance / 2.0, 2.5));

        // Scalars for logging/analysis
        SmartDashboard.putNumber("Shooter/DistanceToHub", distance);
        SmartDashboard.putNumber("Shooter/HoodAngleDeg", hoodDeg);
        SmartDashboard.putNumber("Shooter/TargetRPS", targetRps);
        SmartDashboard.putNumber("Shooter/Aim/Virtual Distance", aim.getLastVirtualDistance());
        SmartDashboard.putBoolean("Shooter/Aim/In Range", aim.isInRange());
    }
}