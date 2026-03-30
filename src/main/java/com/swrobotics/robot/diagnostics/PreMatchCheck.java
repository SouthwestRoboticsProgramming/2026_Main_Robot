package com.swrobotics.robot.diagnostics;

import com.swrobotics.robot.RobotContainer;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public final class PreMatchCheck {
    private static final double UPDATE_INTERVAL = 0.5; // seconds

    private final RobotContainer robot;
    private double lastUpdateTime = 0.0;

    // Track Limelight heartbeats to detect connectivity
    private double lastHbFront = -1;
    private double lastHbRight = -1;
    private double lastHbBack = -1;
    private boolean frontConnected = false;
    private boolean rightConnected = false;
    private boolean backConnected = false;

    public PreMatchCheck(RobotContainer robot) {
        this.robot = robot;
    }

    public void update() {
        double now = Timer.getFPGATimestamp();
        if (now - lastUpdateTime < UPDATE_INTERVAL) return;
        lastUpdateTime = now;

        // Alliance
        boolean allianceSet = DriverStation.getAlliance().isPresent();
        SmartDashboard.putBoolean("Diagnostics/Alliance Set", allianceSet);

        // Limelight connectivity (check heartbeat changes)
        checkLimelight("limelight-f", "Front");
        checkLimelight("limelight-r", "Right");
        checkLimelight("limelight-b", "Back");

        // Shooter telemetry check (if periodic is running, these values will be non-default)
        boolean shooterOk = SmartDashboard.containsKey("Shooter/Flywheel Actual RPS");
        SmartDashboard.putBoolean("Diagnostics/Shooter OK", shooterOk);

        // Hood telemetry check
        boolean hoodOk = SmartDashboard.containsKey("Shooter/Hood Actual Rot");
        SmartDashboard.putBoolean("Diagnostics/Hood OK", hoodOk);

        // Overall
        boolean allOk = allianceSet && shooterOk && hoodOk
                && frontConnected && rightConnected && backConnected;
        SmartDashboard.putBoolean("Diagnostics/ALL OK", allOk);
    }

    private void checkLimelight(String name, String displayName) {
        double hb = NetworkTableInstance.getDefault()
                .getTable(name)
                .getEntry("hb")
                .getDouble(-1);

        boolean connected;
        switch (displayName) {
            case "Front":
                connected = hb != lastHbFront && hb != -1;
                if (hb != -1) lastHbFront = hb;
                frontConnected = connected;
                break;
            case "Right":
                connected = hb != lastHbRight && hb != -1;
                if (hb != -1) lastHbRight = hb;
                rightConnected = connected;
                break;
            case "Back":
                connected = hb != lastHbBack && hb != -1;
                if (hb != -1) lastHbBack = hb;
                backConnected = connected;
                break;
            default:
                connected = false;
        }

        SmartDashboard.putBoolean("Diagnostics/Limelight " + displayName + " OK", connected);
    }
}
