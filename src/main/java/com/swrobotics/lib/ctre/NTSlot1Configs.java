package com.swrobotics.lib.ctre;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.swrobotics.lib.net.NTDouble;
import com.swrobotics.lib.net.NTEntry;

public final class NTSlot1Configs implements TunableConfig {
    // Feedback
    public final NTEntry<Double> kP, kD;
    // Feedforward
    public final NTEntry<Double> kG, kS, kV, kA;

    public NTSlot1Configs(String table, double kP, double kD, double kG, double kS, double kV, double kA) {
        this.kP = new NTDouble(table + "/kP", kP).setPersistent();
        this.kD = new NTDouble(table + "/kD", kD).setPersistent();
        this.kG = new NTDouble(table + "/kG", kG).setPersistent();
        this.kS = new NTDouble(table + "/kS", kS).setPersistent();
        this.kV = new NTDouble(table + "/kV", kV).setPersistent();
        this.kA = new NTDouble(table + "/kA", kA).setPersistent();
    }

    @Override
    public void setAndBind(TalonFXConfiguration config, Runnable applyFn) {
        config.Slot1.kP = kP.get();
        config.Slot1.kD = kD.get();
        config.Slot1.kG = kG.get();
        config.Slot1.kS = kS.get();
        config.Slot1.kV = kV.get();
        config.Slot1.kA = kA.get();

        kP.onChange(() -> { config.Slot1.kP = kP.get(); applyFn.run(); });
        kD.onChange(() -> { config.Slot1.kD = kD.get(); applyFn.run(); });
        kG.onChange(() -> { config.Slot1.kG = kG.get(); applyFn.run(); });
        kS.onChange(() -> { config.Slot1.kS = kS.get(); applyFn.run(); });
        kV.onChange(() -> { config.Slot1.kV = kV.get(); applyFn.run(); });
        kA.onChange(() -> { config.Slot1.kA = kA.get(); applyFn.run(); });
    }

    @Override
    public void setAndBind(TalonFXSConfiguration config, Runnable applyFn) {
        config.Slot1.kP = kP.get();
        config.Slot1.kD = kD.get();
        config.Slot1.kG = kG.get();
        config.Slot1.kS = kS.get();
        config.Slot1.kV = kV.get();
        config.Slot1.kA = kA.get();

        kP.onChange(() -> { config.Slot1.kP = kP.get(); applyFn.run(); });
        kD.onChange(() -> { config.Slot1.kD = kD.get(); applyFn.run(); });
        kG.onChange(() -> { config.Slot1.kG = kG.get(); applyFn.run(); });
        kS.onChange(() -> { config.Slot1.kS = kS.get(); applyFn.run(); });
        kV.onChange(() -> { config.Slot1.kV = kV.get(); applyFn.run(); });
        kA.onChange(() -> { config.Slot1.kA = kA.get(); applyFn.run(); });
    }
}
