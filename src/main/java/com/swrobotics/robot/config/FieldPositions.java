package com.swrobotics.robot.config;

import com.ctre.phoenix6.AllTimestamps;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class FieldPositions {
    //notation: LDS = Left Driver Station, LC = Left left center, RC = Right Center, RDS = Right Driver Station. 
    // -------------------- Hub --------------------
public static final Pose2d kBlueHubPose = new Pose2d(4.687, 4.105, new Rotation2d(0));
public static final Pose2d kRedHubPose  = new Pose2d(4.105,11.853,  new Rotation2d(0));

    // -------------------- Pass -------------------
public static final Pose2d kBluePassPose = new Pose2d(4.2,0, new Rotation2d(0));
public static final Pose2d kRedPassPose = new Pose2d(4.2,18, new Rotation2d(0));

// -------------------- Trench --------------------

// Blue Alliance
public static final Pose2d kLDSBlueTrenchPose = new Pose2d(3.5, 7.5, new Rotation2d(0));
public static final Pose2d kLCBlueTrenchPose  = new Pose2d(6.3, 7.5, new Rotation2d(0));
public static final Pose2d kRCBlueTrenchPose  = new Pose2d(6.3, 0.6, new Rotation2d(0));
public static final Pose2d kRDSBlueTrenchPose = new Pose2d(3.5, 0.6, new Rotation2d(0));

// Red Alliance
public static final Pose2d kLDSRedTrenchPose = new Pose2d(13.3, 7.5, new Rotation2d(0));
public static final Pose2d kLCRedTrenchPose  = new Pose2d(9.5, 7.5, new Rotation2d(0));
public static final Pose2d kRCRedTrenchPose  = new Pose2d(9.5, 0.6, new Rotation2d(0));
public static final Pose2d kRDSRedTrenchPose = new Pose2d(13.3, 0.6, new Rotation2d(0));

// -------------------- Bump --------------------

// Blue Alliance
public static final Pose2d kLDSBlueBumpPose = new Pose2d(3.0, 5.5, new Rotation2d(0));
public static final Pose2d kLCBlueBumpPose  = new Pose2d(6.3, 5.5, new Rotation2d(0));
public static final Pose2d kRCBlueBumpPose  = new Pose2d(6.3, 2.3, new Rotation2d(0));
public static final Pose2d kRDSBlueBumpPose = new Pose2d(3.0, 2.3, new Rotation2d(0));

// Red Alliance
public static final Pose2d kLDSRedBumpPose = new Pose2d(13.3, 5.5, new Rotation2d(0));
public static final Pose2d kLCRedBumpPose  = new Pose2d(9.5, 5.5, new Rotation2d(0));
public static final Pose2d kRCRedBumpPose  = new Pose2d(9.5, 2.3, new Rotation2d(0));
public static final Pose2d kRDSRedBumpPose = new Pose2d(13.3, 2.3, new Rotation2d(0));

// -------------------- Alliance-based Accessors --------------------
public static Pose2d getAllianceHubPose(Alliance alliance) {
return alliance == Alliance.Blue ? kBlueHubPose : kRedHubPose;
}

public static Pose2d getAlliancePassPose(Alliance alliance){
return alliance == Alliance.Blue ? kBluePassPose : kRedPassPose; 
}

}