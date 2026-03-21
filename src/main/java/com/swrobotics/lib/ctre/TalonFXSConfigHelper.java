package com.swrobotics.lib.ctre;

import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.hardware.TalonFXS;

import java.util.ArrayList;
import java.util.List;

public final class TalonFXSConfigHelper extends TalonFXSConfiguration {
    private final List<TalonFXS> motors;

    public TalonFXSConfigHelper() {
        motors = new ArrayList<>();

        this.Audio.BeepOnBoot = true;
        this.Audio.BeepOnConfig = true;
        this.Audio.AllowMusicDurDisable = true;
    }

    public TalonFXSConfigHelper addTunable(TunableConfig tunable) {
        tunable.setAndBind(this, () -> {
            for (TalonFXS motor : motors) {
                CTREUtil.retryUntilOk(motor, () -> motor.getConfigurator().apply(this));
            }
        });
        return this;
    }

    public void apply(TalonFXS... motors) {
        for (TalonFXS fxs : motors) {
            this.motors.add(fxs);
            CTREUtil.retryUntilOk(fxs, () -> fxs.getConfigurator().apply(this));
        }
    }

    public void reapply() {
        for (TalonFXS fxs : motors) {
            CTREUtil.retryUntilOk(fxs, () -> fxs.getConfigurator().apply(this));
        }
    }
}
