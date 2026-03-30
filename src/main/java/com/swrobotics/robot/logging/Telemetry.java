package com.swrobotics.robot.logging;

import com.swrobotics.robot.control.AimCalc;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public final class Telemetry {
    private static final Mechanism2d mech = new Mechanism2d(3.0, 3.0);
    private static final MechanismRoot2d root = mech.getRoot("shooter", 1.5, 0.2);
    private static final MechanismLigament2d hoodLig = root.append(new MechanismLigament2d("hood", 1.0, 0.0));
    
    private Telemetry() {}

    public static void init() {
        // Starts the WPILib automatic data logger (logs all SmartDashboard values to flash drive)
        DataLogManager.start();
        SmartDashboard.putData("Shooter/Mech2d", mech);
    }

    public static void periodic() {
        AimCalc aim = AimCalc.getInstance();

        double distance = aim.getLastDistance();
        double targetHoodDeg = aim.getHoodAngle(false).getDegrees();
        double targetRps = aim.getShooterRPS();

        // Update visuals
        hoodLig.setAngle(targetHoodDeg);

        // Logging to SmartDashboard (and automatically to DataLog)
        SmartDashboard.putNumber("Shooter/Telemetry/Distance", distance);
        SmartDashboard.putNumber("Shooter/Telemetry/TargetHoodDeg", targetHoodDeg);
        SmartDashboard.putNumber("Shooter/Telemetry/TargetRPS", targetRps);
        SmartDashboard.putBoolean("Shooter/Telemetry/InRange", aim.isInRange());
    }

   
    public static Command logSuccessfulShot() {
        return Commands.runOnce(() -> {
            AimCalc aim = AimCalc.getInstance();
            double dist = aim.getLastDistance();
            double ang = aim.getHoodAngle(false).getDegrees();
            double rps = aim.getShooterRPS();

            DataLogManager.log(String.format("SUCCESSFUL SHOT: Dist: %.2f, Angle: %.2f, RPS: %.2f", dist, ang, rps));

            aim.saveCurrentShot(dist, ang, rps);
        }).ignoringDisable(true);
    }
}