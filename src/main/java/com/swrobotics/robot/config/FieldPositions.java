package com.swrobotics.robot.config;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class FieldPositions {
    
    public static final Pose2d kBlueHubPose = new Pose2d(4.687, 4.105, new Rotation2d(0));
    public static final Pose2d kRedHubPose = new Pose2d(12.0, 4.105, new Rotation2d(0));

    public static Pose2d getAllianceHubPose(Alliance alliance){
        return alliance == Alliance.Blue ? kBlueHubPose : kRedHubPose;
        
    }
}