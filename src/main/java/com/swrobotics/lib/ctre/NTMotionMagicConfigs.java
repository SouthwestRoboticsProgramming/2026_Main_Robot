package com.swrobotics.lib.ctre;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.swrobotics.lib.net.NTDouble;
import com.swrobotics.lib.net.NTEntry;

public final class NTMotionMagicConfigs implements TunableConfig {
    private final NTEntry<Double> cruiseVelocity;
    private final NTEntry<Double> acceleration;
    private final NTEntry<Double> jerk;

    public NTMotionMagicConfigs(String table, double cruiseVelocity, double acceleration, double jerk) {
        this.cruiseVelocity = new NTDouble(table + "/Cruise Velocity", cruiseVelocity).setPersistent();
        this.acceleration = new NTDouble(table + "/Acceleration", acceleration).setPersistent();
        this.jerk = new NTDouble(table + "/Jerk", jerk).setPersistent();
    }

    @Override
    public void setAndBind(TalonFXConfiguration config, Runnable applyFn) {
        config.MotionMagic.MotionMagicCruiseVelocity = cruiseVelocity.get();
        config.MotionMagic.MotionMagicAcceleration = acceleration.get();
        config.MotionMagic.MotionMagicJerk = jerk.get();

        cruiseVelocity.onChange(() -> {
            config.MotionMagic.MotionMagicCruiseVelocity = cruiseVelocity.get();
            applyFn.run();
        });
        acceleration.onChange(() -> {
            config.MotionMagic.MotionMagicAcceleration = acceleration.get();
            applyFn.run();
        });
        jerk.onChange(() -> {
            config.MotionMagic.MotionMagicJerk = jerk.get();
            applyFn.run();
        });
    }

    @Override
    public void setAndBind(TalonFXSConfiguration config, Runnable applyFn) {
        config.MotionMagic.MotionMagicCruiseVelocity = cruiseVelocity.get();
        config.MotionMagic.MotionMagicAcceleration = acceleration.get();
        config.MotionMagic.MotionMagicJerk = jerk.get();

        cruiseVelocity.onChange(() -> {
            config.MotionMagic.MotionMagicCruiseVelocity = cruiseVelocity.get();
            applyFn.run();
        });
        acceleration.onChange(() -> {
            config.MotionMagic.MotionMagicAcceleration = acceleration.get();
            applyFn.run();
        });
        jerk.onChange(() -> {
            config.MotionMagic.MotionMagicJerk = jerk.get();
            applyFn.run();
        });
    }

    public double getCruiseVelocity() {
        return cruiseVelocity.get();
    }

    public double getAcceleration() {
        return acceleration.get();
    }

    public double getJerk() {
        return jerk.get();
    }
}
