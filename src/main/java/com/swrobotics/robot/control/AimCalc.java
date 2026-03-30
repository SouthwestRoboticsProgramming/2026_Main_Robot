package com.swrobotics.robot.control;

import com.swrobotics.lib.net.NTBoolean;
import com.swrobotics.lib.net.NTDouble;
import com.swrobotics.lib.net.NTEntry;
import com.swrobotics.lib.net.NTInteger;
import com.swrobotics.robot.config.Constants;
import com.swrobotics.robot.config.FieldPositions;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.DriverStation;

public class AimCalc {
    private static final AimCalc instance = new AimCalc();
    public static AimCalc getInstance() { return instance; }

    // Number of interpolation data points (tune count here if needed)
    private static final int NUM_POINTS = 6;

    // Default (distance_m, hood_angle_deg) pairs — starting guesses, tune on field
    private static final double[][] DEFAULT_HOOD_POINTS = {
        {1.0,  25.0},
        {1.5,  28.0},
        {2.0,  31.0},
        {2.5,  34.0},
        {3.0,  37.0},
        {3.5,  40.0},
    };

    // Default (distance_m, shooter_rps) pairs — starting guesses, tune on field
    private static final double[][] DEFAULT_RPS_POINTS = {
        {1.0,  18.0},
        {1.5,  19.0},
        {2.0,  20.0},
        {2.5,  21.0},
        {3.0,  22.0},
        {3.5,  24.0},
    };

    // NT-tunable data points for hood angle interpolation
    private final NTEntry<Double>[] hoodDists = new NTEntry[NUM_POINTS];
    private final NTEntry<Double>[] hoodAngles = new NTEntry[NUM_POINTS];

    // NT-tunable data points for shooter RPS interpolation
    private final NTEntry<Double>[] rpsDists = new NTEntry[NUM_POINTS];
    private final NTEntry<Double>[] rpsValues = new NTEntry[NUM_POINTS];

    // Interpolation maps rebuilt from NT values
    private InterpolatingDoubleTreeMap hoodMap = new InterpolatingDoubleTreeMap();
    private InterpolatingDoubleTreeMap rpsMap = new InterpolatingDoubleTreeMap();

    // Output values
    private Rotation2d drivebaseAimAngle = new Rotation2d();
    private Rotation2d hoodAngle = new Rotation2d();
    private double shooterRPS = 20.0;
    private double lastDistanceToHub = 0.0;
    private double lastVirtualDistance = 0.0;
    private boolean inRange = false;

    // Track min/max keys for range checking
    private double hoodMapMinDist = 0.0;
    private double hoodMapMaxDist = 0.0;

    // --- Tuning helpers ---
    // "Save Shot" button: records current (distance, hood angle, RPS) into the next point slot
    private final NTEntry<Boolean> saveShotTrigger = new NTBoolean("Shooter/Aim/Tuning/Save Shot", false);
    private final NTEntry<Double> nextSaveSlot = new NTDouble("Shooter/Aim/Tuning/Next Save Slot", 0);
    // "Lock Distance" toggle: freezes distance reading so it doesn't jitter while tuning
    private final NTEntry<Boolean> lockDistance = new NTBoolean("Shooter/Aim/Tuning/Lock Distance", false);
    private double lockedDistance = 0.0;

    private AimCalc() {
        // Create NT entries for hood angle data points
        for (int i = 0; i < NUM_POINTS; i++) {
            hoodDists[i] = new NTDouble("Shooter/Aim/Hood Point " + i + " Dist (m)", DEFAULT_HOOD_POINTS[i][0]).setPersistent();
            hoodAngles[i] = new NTDouble("Shooter/Aim/Hood Point " + i + " Angle (deg)", DEFAULT_HOOD_POINTS[i][1]).setPersistent();

            rpsDists[i] = new NTDouble("Shooter/Aim/RPS Point " + i + " Dist (m)", DEFAULT_RPS_POINTS[i][0]).setPersistent();
            rpsValues[i] = new NTDouble("Shooter/Aim/RPS Point " + i + " RPS", DEFAULT_RPS_POINTS[i][1]).setPersistent();

            // Rebuild maps when any value changes
            hoodDists[i].onChange(this::rebuildMaps);
            hoodAngles[i].onChange(this::rebuildMaps);
            rpsDists[i].onChange(this::rebuildMaps);
            rpsValues[i].onChange(this::rebuildMaps);
        }

        // Also rebuild when offset changes
        Constants.kHoodAngleOffset.onChange(this::rebuildMaps);

        // Save shot: when toggled true, record current state into next slot
        saveShotTrigger.onChange(() -> {
            if (saveShotTrigger.get()) {
                saveCurrentShot();
                saveShotTrigger.set(false);
            }
        });

        rebuildMaps();
    }

    /**
     * Records the current distance, hood angle, and shooter RPS into the next
     * available interpolation point slot. Called from the "Save Shot" button.
     */
    public void saveCurrentShot(double distanceM, double angleDeg, double rps) {
        int slot = (int) Math.round(nextSaveSlot.get());
        if (slot < 0 || slot >= NUM_POINTS) {
            slot = 0; // Wrap around
        }

        hoodDists[slot].set(distanceM);
        hoodAngles[slot].set(angleDeg);
        rpsDists[slot].set(distanceM);
        rpsValues[slot].set(rps);

        // Advance to next slot
        nextSaveSlot.set((double) ((slot + 1) % NUM_POINTS));
        rebuildMaps();
    }

    /** Save using current live values (called from NT button) */
    private void saveCurrentShot() {
        saveCurrentShot(lastDistanceToHub, currentHoodAngleDeg, shooterRPS);
    }

    /** Set by HoodSubsystem so save-shot can capture the actual hood angle */
    private double currentHoodAngleDeg = 0.0;
    public void setCurrentHoodAngleDeg(double deg) {
        currentHoodAngleDeg = deg;
    }

    private void rebuildMaps() {
        InterpolatingDoubleTreeMap newHoodMap = new InterpolatingDoubleTreeMap();
        InterpolatingDoubleTreeMap newRpsMap = new InterpolatingDoubleTreeMap();

        double minDist = Double.MAX_VALUE;
        double maxDist = -Double.MAX_VALUE;

        for (int i = 0; i < NUM_POINTS; i++) {
            double dist = hoodDists[i].get();
            double angle = hoodAngles[i].get();
            newHoodMap.put(dist, angle);

            minDist = Math.min(minDist, dist);
            maxDist = Math.max(maxDist, dist);
        }

        for (int i = 0; i < NUM_POINTS; i++) {
            double dist = rpsDists[i].get();
            double rps = rpsValues[i].get();
            newRpsMap.put(dist, rps);
        }

        hoodMap = newHoodMap;
        rpsMap = newRpsMap;
        hoodMapMinDist = minDist;
        hoodMapMaxDist = maxDist;
    }

    public void update(Pose2d robotPose, double fieldVx, double fieldVy) {
        // Alliance-relative hub position
        Translation2d hubTarget = FieldPositions.getAllianceHubPose(
            DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue)
        ).getTranslation();

        // Shooter offset from robot center (rotated to field frame)
        Translation2d shooterOffset = new Translation2d(
            Constants.kShooterOffsetX,
            Constants.kShooterOffsetY
        ).rotateBy(robotPose.getRotation());

        Translation2d shooterFieldPos = robotPose.getTranslation().plus(shooterOffset);

        // Static distance from shooter to hub
        double rawDist = shooterFieldPos.getDistance(hubTarget);

        // Lock distance: freeze the reading for stable tuning
        if (lockDistance.get()) {
            if (lockedDistance == 0.0) {
                lockedDistance = rawDist; // Capture on first lock
            }
        } else {
            lockedDistance = rawDist;
        }
        double staticDist = lockedDistance;
        lastDistanceToHub = staticDist;

        // Iterative shoot-on-the-move compensation (converges in ~5 iterations)
        double baseFlightTime = Constants.kBaseFlightTime.get();
        double flightTimePerMeter = Constants.kFlightTimePerMeter.get();

        Translation2d virtualTarget = hubTarget;
        double vDist = staticDist;

        for (int i = 0; i < 5; i++) {
            double shotTime = baseFlightTime + flightTimePerMeter * vDist;
            Translation2d robotMotionDuringShot = new Translation2d(
                fieldVx * shotTime,
                fieldVy * shotTime
            );
            virtualTarget = hubTarget.minus(robotMotionDuringShot);
            Translation2d delta = virtualTarget.minus(shooterFieldPos);
            vDist = delta.getNorm();
        }

        lastVirtualDistance = vDist;

        // Drivebase aim angle (direction robot should face)
        Translation2d delta = virtualTarget.minus(shooterFieldPos);
        drivebaseAimAngle = delta.getAngle();

        // Check if distance is within the interpolation table range
        inRange = vDist >= hoodMapMinDist && vDist <= hoodMapMaxDist;

        // Look up hood angle from interpolation table + apply global offset
        double hoodDeg = hoodMap.get(vDist) + Constants.kHoodAngleOffset.get();
        hoodAngle = Rotation2d.fromDegrees(hoodDeg);

        // Look up shooter RPS from interpolation table
        shooterRPS = rpsMap.get(vDist);
    }

    public Rotation2d getDrivebaseAimAngle() { return drivebaseAimAngle; }
    public Rotation2d getHoodAngle() { return hoodAngle; }
    public double getShooterRPS() { return shooterRPS; }
    public double getLastDistanceToHub() { return lastDistanceToHub; }
    public double getLastVirtualDistance() { return lastVirtualDistance; }
    public boolean isInRange() { return inRange; }
}
