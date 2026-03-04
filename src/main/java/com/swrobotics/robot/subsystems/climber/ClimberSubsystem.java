package com.swrobotics.robot.subsystems.climber;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.swrobotics.lib.ctre.TalonFXConfigHelper;
import com.swrobotics.lib.net.NTBoolean;
import com.swrobotics.lib.net.NTEntry;
import com.swrobotics.robot.config.Constants;
import com.swrobotics.robot.config.IOAllocation;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public final class ClimberSubsystem extends SubsystemBase {
    public enum State {
        RETRACTED,
        EXTENDED
    }

    private final TalonFX motor;
    private final StatusSignal<AngularVelocity> motorVelocity;
    private final StatusSignal<Angle> motorPosition;

    private boolean hasCalibrated;
    private Debouncer calibrationDebounce;
    private State targetState;

    private double targetPos;
    private double manualAdjust;

    private NTEntry<Boolean> calibrating = new NTBoolean("Climber/Calibrating?", true);

    public ClimberSubsystem() {
        motor = IOAllocation.CAN.kClimberMotor.createTalonFX();
        
        // --- HARDWARE CONFIGURATION ---
        TalonFXConfigHelper config = new TalonFXConfigHelper();

        
        // 1. Current Limiting: Crucial for 130lb lift
        // Limits supply to 40A continuous, 60A peak to prevent brownouts
        CurrentLimitsConfigs limits = new CurrentLimitsConfigs();
        limits.StatorCurrentLimit = 80.0; // Allow high torque for the lift
        limits.StatorCurrentLimitEnable = true;
        limits.SupplyCurrentLimit = 40.0; // Protect the battery/main breaker
        limits.SupplyCurrentLimitEnable = true;
        motor.getConfigurator().apply(limits);

        // 2. Brake Mode: Ensures we don't drop after the match ends
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        config.addTunable(Constants.kClimberPID);


        motor.getConfigurator().apply(config);
        
        motorVelocity = motor.getVelocity();
        motorPosition = motor.getPosition();

        hasCalibrated = RobotBase.isSimulation();
        targetState = State.RETRACTED;
        targetPos = 0;
        manualAdjust = 0;

        // Default to retracted but ONLY after calibration logic handles it
        setDefaultCommand(commandSetState(State.RETRACTED));
    }

    public void setState(State state) {
        targetState = state;
        if (!hasCalibrated) return;

        // Retracted should be slightly above hard-stop (e.g., 0.5 rotations) 
        // to avoid stalling the motor against the frame constantly
        double position = (state == State.EXTENDED) 
                ? Constants.kClimberTall.get() 
                : 0.5; 

        if (state == State.EXTENDED)
            position += manualAdjust;

        // Use PositionVoltage with FeedForward if necessary to fight gravity
        motor.setControl(new PositionVoltage(position).withSlot(0));
        targetPos = position;
    }

    private double calibrationStartTime = -1;

@Override
public void periodic() {
    motorPosition.refresh();
    motorVelocity.refresh();

    if (DriverStation.isDisabled()) {
        hasCalibrated = false; // Force re-calibration on next enable
        calibrationStartTime = -1;
        return;
    }

    if (!hasCalibrated) {
        if (calibrationStartTime == -1) {
            calibrationStartTime = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
        }
        runCalibration();
    }
}

private void runCalibration() {
    if (calibrationDebounce == null) {
        calibrationDebounce = new Debouncer(
            Constants.kClimberCalibrationTime.get(),
            Debouncer.DebounceType.kBoth
        );
    }

    double velocity = motorVelocity.getValueAsDouble();
    double currentTime = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
    
    // Condition 1: Motor has stopped moving (hit hardstop)
    boolean reachedHardStop = Math.abs(velocity) < Constants.kClimberCalibrationVelocity.get();
    
    // Condition 2: 5 seconds have passed since enable
    boolean timedOut = (currentTime - calibrationStartTime) > 8.5;

    if (calibrationDebounce.calculate(reachedHardStop) || timedOut) {
        // Use the "CalibrationPosition" (usually -0.5 or 0) to set the zero point
        motor.setPosition(Constants.kClimberCalibrationPosition.get());
        hasCalibrated = true;
        calibrating.set(false);
        
        // Immediately switch to holding the retracted state
        setState(State.RETRACTED);
    } else {
        calibrating.set(true);
        // Pull down safely to find the bottom
        motor.setControl(new VoltageOut(-Constants.kClimberCalibrationVoltage.get()));
    }
}

    public Command commandSetState(State state) {
        return Commands.run(() -> setState(state), this);
    }

    public void manualAdjust(double adjustment) {
        if (!hasCalibrated) return; // Don't allow manual adjust until calibrated
        manualAdjust += adjustment;
        setState(targetState); // Re-apply target state to update position with new adjust
    }
}