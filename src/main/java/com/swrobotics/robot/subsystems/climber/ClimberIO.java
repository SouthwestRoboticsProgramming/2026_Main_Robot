package com.swrobotics.robot.subsystems.climber;

import org.littletonrobotics.junction.AutoLog;

public interface ClimberIO {
    @AutoLog
    public static class ClimberIOInputs {
        public double positionRotations = 0.0;
        public double velocityRPS = 0.0;
        public double appliedVolts = 0.0;
        public double supplyCurrentAmps = 0.0;
        public double statorCurrentAmps = 0.0;
        public double tempCelsius = 0.0;
    }

    public default void updateInputs(ClimberIOInputs inputs) {}

    public default void setPosition(double rotations) {}

    public default void setVoltage(double volts) {}

    public default void stop() {}

    public default void zeroPosition(double position) {}
}
