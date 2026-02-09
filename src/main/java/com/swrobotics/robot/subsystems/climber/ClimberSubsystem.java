package com.swrobotics.robot.subsystems.climber;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimberSubsystem extends SubsystemBase{
    public enum State {
        BOTTOM,
        SHORTCLIMB,
        HIGHCLIMB
    }
}