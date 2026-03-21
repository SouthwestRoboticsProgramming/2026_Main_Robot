package com.swrobotics.lib.ctre;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.swrobotics.lib.net.NTDouble;
import com.swrobotics.lib.net.NTEntry;

public final class NTSlot0Configs implements TunableConfig {
    // Feedback
    public final NTEntry<Double> kP, kD;
    // Feedforward
    public final NTEntry<Double> kG, kS, kV, kA;

    public NTSlot0Configs(String table, double kP, double kD, double kG, double kS, double kV, double kA) {
        this.kP = new NTDouble(table + "/kP", kP).setPersistent();
        this.kD = new NTDouble(table + "/kD", kD).setPersistent();
        this.kG = new NTDouble(table + "/kG", kG).setPersistent();
        this.kS = new NTDouble(table + "/kS", kS).setPersistent();
        this.kV = new NTDouble(table + "/kV", kV).setPersistent();
        this.kA = new NTDouble(table + "/kA", kA).setPersistent();
    }

    @Override
    public void setAndBind(TalonFXConfiguration config, Runnable applyFn) {
        config.Slot0.kP = kP.get();
        config.Slot0.kD = kD.get();
        config.Slot0.kG = kG.get();
        config.Slot0.kS = kS.get();
        config.Slot0.kV = kV.get();
        config.Slot0.kA = kA.get();

        kP.onChange(() -> { config.Slot0.kP = kP.get(); applyFn.run(); });
        kD.onChange(() -> { config.Slot0.kD = kD.get(); applyFn.run(); });
        kG.onChange(() -> { config.Slot0.kG = kG.get(); applyFn.run(); });
        kS.onChange(() -> { config.Slot0.kS = kS.get(); applyFn.run(); });
        kV.onChange(() -> { config.Slot0.kV = kV.get(); applyFn.run(); });
        kA.onChange(() -> { config.Slot0.kA = kA.get(); applyFn.run(); });
    }

    @Override
    public void setAndBind(TalonFXSConfiguration config, Runnable applyFn) {
        config.Slot0.kP = kP.get();
        config.Slot0.kD = kD.get();
        config.Slot0.kG = kG.get();
        config.Slot0.kS = kS.get();
        config.Slot0.kV = kV.get();
        config.Slot0.kA = kA.get();

        kP.onChange(() -> { config.Slot0.kP = kP.get(); applyFn.run(); });
        kD.onChange(() -> { config.Slot0.kD = kD.get(); applyFn.run(); });
        kG.onChange(() -> { config.Slot0.kG = kG.get(); applyFn.run(); });
        kS.onChange(() -> { config.Slot0.kS = kS.get(); applyFn.run(); });
        kV.onChange(() -> { config.Slot0.kV = kV.get(); applyFn.run(); });
        kA.onChange(() -> { config.Slot0.kA = kA.get(); applyFn.run(); });
    }
}
