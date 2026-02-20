package com.swrobotics.robot.subsystems.intake.expansions;

import org.littletonrobotics.junction.AutoLog;

public interface ExpansionIO {
    @AutoLog
    public static class ExpansionIOInputs {
        public double positionRotations = 0.0;
        public double velocityRPS = 0.0;
        public double appliedVolts = 0.0;
        public double supplyCurrentAmps = 0.0;
        public double statorCurrentAmps = 0.0;
        public double tempCelsius = 0.0;
    }

    public default void updateInputs(ExpansionIOInputs inputs) {}

    public default void setPosition(double rotations) {}

    public default void stop() {}

    public default void zeroPosition() {}
}
