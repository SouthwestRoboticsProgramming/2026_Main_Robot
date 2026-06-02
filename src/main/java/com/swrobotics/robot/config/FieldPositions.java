package com.swrobotics.robot.config;

import com.swrobotics.lib.field.FieldInfo;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

/**
 * Constants governing the physical layout of the playing field, target configurations,
 * match notifications, and alliance coordinate maps.
 */
public final class FieldPositions {
    // Field Identification
    public static final FieldInfo kField = FieldInfo.REBUILT_2026;
    public static final Pose2d kHubPose = new Pose2d(8.267, 4.105, new Rotation2d(0));
    public static final double kHubHeightMeters = Units.inchesToMeters(49.5); // m

    // Match Timer Alerts (Seconds Remaining)
    public static final int kEndgameAlertTime = 20;
    public static final int kEndgameAlert2Time = 5;
    public static final int kTransferAlertTime = 130;
    
    // Status Iteration Alerts
    public static final int kActive_InactiveAlert1Time = 120;
    public static final int kActive_InactiveAlert1Time2 = 115;
    public static final int kActive_InactiveAlert2Time = 95;
    public static final int kActive_InactiveAlert2Time2 = 90;
    public static final int kActive_InactiveAlert3Time = 70;
    public static final int kActive_InactiveAlert3Time2 = 65;
    public static final int kActive_InactiveAlert4Time = 35;
    public static final int kActive_InactiveAlert4Time2 = 30;

    // Notation key: LDS = Left Driver Station, LC = Left Center, RC = Right Center, RDS = Right Driver Station.
    
    // Target Hub Layout Locations
    public static final Pose2d kBlueHubPose = new Pose2d(4.7, 4, new Rotation2d(0));
    public static final Pose2d kRedHubPose  = new Pose2d(12, 4, new Rotation2d(0));

    // Launch Pass Boundary Lines
    public static final Pose2d kBluePassPose = new Pose2d(4.2, 0, new Rotation2d(0));
    public static final Pose2d kRedPassPose = new Pose2d(4.2, 18, new Rotation2d(0));

    // Trench Safe-Zones
    // Blue Alliance
    public static final Pose2d kLDSBlueTrenchPose = new Pose2d(3.0, 7.5, new Rotation2d(0));
    public static final Pose2d kLCBlueTrenchPose  = new Pose2d(6.3, 7.5, new Rotation2d(0));
    public static final Pose2d kRCBlueTrenchPose  = new Pose2d(6.3, 0.6, new Rotation2d(0));
    public static final Pose2d kRDSBlueTrenchPose = new Pose2d(3.0, 0.6, new Rotation2d(0));
    // Red Alliance
    public static final Pose2d kLDSRedTrenchPose = new Pose2d(12.8, 7.5, new Rotation2d(0));
    public static final Pose2d kLCRedTrenchPose  = new Pose2d(9.5, 7.5, new Rotation2d(0));
    public static final Pose2d kRCRedTrenchPose  = new Pose2d(9.5, 0.6, new Rotation2d(0));
    public static final Pose2d kRDSRedTrenchPose = new Pose2d(12.8, 0.6, new Rotation2d(0));

    // Field Floor Obstacle Bumps
    // Blue Alliance
    public static final Pose2d kLDSBlueBumpPose = new Pose2d(3.0, 5.5, new Rotation2d(0));
    public static final Pose2d kLCBlueBumpPose  = new Pose2d(6.3, 5.5, new Rotation2d(0));
    public static final Pose2d kRCBlueBumpPose  = new Pose2d(6.3, 2.3, new Rotation2d(0));
    public static final Pose2d kRDSBlueBumpPose = new Pose2d(3.0, 2.3, new Rotation2d(0));
    // Red Alliance
    public static final Pose2d kLDSRedBumpPose = new Pose2d(13.8, 5.5, new Rotation2d(0));
    public static final Pose2d kLCRedBumpPose  = new Pose2d(9.5, 5.5, new Rotation2d(0));
    public static final Pose2d kRCRedBumpPose  = new Pose2d(9.5, 2.3, new Rotation2d(0));
    public static final Pose2d kRDSRedBumpPose = new Pose2d(13.8, 2.3, new Rotation2d(0));

    // Alliance-Based Relative Accessors
    public static Pose2d getAllianceHubPose(Alliance alliance) {
        return alliance == Alliance.Blue ? kBlueHubPose : kRedHubPose;
    }

    public static Pose2d getAlliancePassPose(Alliance alliance) {
        return alliance == Alliance.Blue ? kBluePassPose : kRedPassPose; 
    }

    private FieldPositions() {
        throw new UnsupportedOperationException("This is a constant data file and cannot be instantiated");
    }
}