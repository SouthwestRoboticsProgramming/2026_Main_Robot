package com.swrobotics.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
    @AutoLog
    public static class IntakeIOInputs {
        public double velocityRPS = 0.0;
        public double appliedVolts = 0.0;
        public double supplyCurrentAmps = 0.0;
        public double statorCurrentAmps = 0.0;
        public double tempCelsius = 0.0;
    }

    public default void updateInputs(IntakeIOInputs inputs) {}

    public default void setVelocity(double rps) {}

    public default void stop() {}
}
