package com.swrobotics.robot.subsystems.indexer;

import org.littletonrobotics.junction.AutoLog;

public interface IndexerIO {
    @AutoLog
    public static class IndexerIOInputs {
        public double motor1VelocityRPS = 0.0;
        public double motor2VelocityRPS = 0.0;
        public double motor1AppliedVolts = 0.0;
        public double motor2AppliedVolts = 0.0;
        public double motor1CurrentAmps = 0.0;
        public double motor2CurrentAmps = 0.0;
        public double motor1TempCelsius = 0.0;
        public double motor2TempCelsius = 0.0;
        public boolean ballDetected = false;
    }

    public default void updateInputs(IndexerIOInputs inputs) {}

    public default void setMotor1Velocity(double rps) {}

    public default void setMotor2Velocity(double rps) {}

    public default void stop() {}
}
