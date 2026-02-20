package com.swrobotics.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
    @AutoLog
    public static class ShooterIOInputs {
        public double velocityRPS = 0.0;
        public double appliedVolts = 0.0;
        public double supplyCurrentAmps = 0.0;
        public double statorCurrentAmps = 0.0;
        public double tempCelsius = 0.0;
    }

    public default void updateInputs(ShooterIOInputs inputs) {}

    public default void setVelocity(double rps) {}

    public default void stop() {}
}
