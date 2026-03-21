package com.swrobotics.lib.ctre;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;

public interface TunableConfig {
    void setAndBind(TalonFXConfiguration config, Runnable applyFn);
    void setAndBind(TalonFXSConfiguration config, Runnable applyFn);
}
