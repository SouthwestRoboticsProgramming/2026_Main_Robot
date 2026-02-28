package com.swrobotics.robot.config;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class FieldPositions {
    //notation: LDS = Left Driver Station, LC = Left left center, RC = Right Center, RDS = Right Driver Station. 
    // -------------------- Hub --------------------
public static final Pose2d kBlueHubPose = new Pose2d(4.687, 4.105, new Rotation2d(0));
public static final Pose2d kRedHubPose  = new Pose2d(11.853, 4.105, new Rotation2d(0));

// -------------------- Trench --------------------
// Blue Alliance
public static final Pose2d kLDSBlueTrenchPose = new Pose2d(3.5, 7.5, new Rotation2d(0));
public static final Pose2d kLCBlueTrenchPose  = new Pose2d(6.0, 7.5, new Rotation2d(0));
public static final Pose2d kRCBlueTrenchPose  = new Pose2d(2.0, 2.0, new Rotation2d(0));
public static final Pose2d kRDSBlueTrenchPose = new Pose2d(2.0, 2.0, new Rotation2d(0));

// Red Alliance
public static final Pose2d kLDSRedTrenchPose = new Pose2d(6.0, 7.5, new Rotation2d(0));
public static final Pose2d kLCRedTrenchPose  = new Pose2d(13.0, 4.105, new Rotation2d(0));
public static final Pose2d kRCRedTrenchPose  = new Pose2d(14.5, 7.5, new Rotation2d(0));
public static final Pose2d kRDSRedTrenchPose = new Pose2d(14.5, 7.5, new Rotation2d(0));

// -------------------- Bump --------------------
// Blue Alliance
public static final Pose2d kLDSBlueBumpPose = new Pose2d(1.0, 4.105, new Rotation2d(0));
public static final Pose2d kLCBlueBumpPose  = new Pose2d(1.0, 4.105, new Rotation2d(0));
public static final Pose2d kRCBlueBumpPose  = new Pose2d(2.0, 2.0, new Rotation2d(0));
public static final Pose2d kRDSBlueBumpPose = new Pose2d(2.0, 2.0, new Rotation2d(0));

// Red Alliance
public static final Pose2d kLDSRedBumpPose = new Pose2d(13.0, 4.105, new Rotation2d(0));
public static final Pose2d kLCRedBumpPose  = new Pose2d(13.0, 4.105, new Rotation2d(0));
public static final Pose2d kRCRedBumpPose  = new Pose2d(14.5, 6.0, new Rotation2d(0));
public static final Pose2d kRDSRedBumpPose = new Pose2d(14.5, 6.0, new Rotation2d(0));

// -------------------- Alliance-based Accessors --------------------
public static Pose2d getAllianceHubPose(Alliance alliance) {
return alliance == Alliance.Blue ? kBlueHubPose : kRedHubPose;
}
public static Pose2d getAllianceLDSTrenchPose(Alliance alliance) {
    return alliance == Alliance.Blue ? kLDSBlueTrenchPose : kLDSRedTrenchPose;
}
public static Pose2d getAllianceLCTrenchPose(Alliance alliance) {
    return alliance == Alliance.Blue ? kLCBlueTrenchPose : kLCRedTrenchPose;
}
public static Pose2d getAllianceRCTrenchPose(Alliance alliance) {
    return alliance == Alliance.Blue ? kRCBlueTrenchPose : kRCRedTrenchPose;
}
public static Pose2d getAllianceRDSTrenchPose(Alliance alliance) {
    return alliance == Alliance.Blue ? kRDSBlueTrenchPose : kRDSRedTrenchPose;
}
public static Pose2d getAllianceLDSBumpPose(Alliance alliance) {
    return alliance == Alliance.Blue ? kLDSBlueBumpPose : kLDSRedBumpPose;
}
public static Pose2d getAllianceLCBumpPose(Alliance alliance) {
    return alliance == Alliance.Blue ? kLCBlueBumpPose : kLCRedBumpPose;
}
public static Pose2d getAllianceRCBumpPose(Alliance alliance) {
    return alliance == Alliance.Blue ? kRCBlueBumpPose : kRCRedBumpPose;
}
public static Pose2d getAllianceRDSBumpPose(Alliance alliance) {
    return alliance == Alliance.Blue ? kRDSBlueBumpPose : kRDSRedBumpPose;
}
}