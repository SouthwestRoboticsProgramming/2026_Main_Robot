package com.swrobotics.robot.subsystems.Hood;

import org.littletonrobotics.junction.AutoLog;

public interface HoodIO {
    @AutoLog
    public static class HoodIOInputs {
        public double positionDeg = 0.0;
        public double velocityDegPerSec = 0.0;
        public double appliedVolts = 0.0;
        public double supplyCurrentAmps = 0.0;
        public double statorCurrentAmps = 0.0;
        public double tempCelsius = 0.0;
        public double absolutePositionDeg = 0.0;
    }

    public default void updateInputs(HoodIOInputs inputs) {}

    public default void setPosition(double motorRotations) {}

    public default void stop() {}

    public default void seedPosition(double rotations) {}
}
