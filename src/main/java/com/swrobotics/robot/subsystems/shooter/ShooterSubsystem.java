package com.swrobotics.robot.subsystems.shooter;

import com.ctre.phoenix6.controls.VelocityVoltage; // Switched to Velocity
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import com.swrobotics.lib.ctre.TalonFXConfigHelper;
import com.swrobotics.robot.config.Constants;
import com.swrobotics.robot.config.IOAllocation;
import com.swrobotics.robot.control.AimCalc;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.function.DoubleSupplier;

public class ShooterSubsystem extends SubsystemBase {

    public enum State {
        IDLE,
        SHOOT,
        WARM, // Shooter is at a lower speed to warm up but not shoot
        RINDEX // Shooter is running in reverse to help unjam balls or index them backwards
    }
    
    private final TalonFX motorL;
    private final TalonFX motorR;

    private final VelocityVoltage velocityControl = new VelocityVoltage(0);
    private State targetState;

    public ShooterSubsystem() {

        motorL = IOAllocation.CAN.kShooterL.createTalonFX();
        motorR = IOAllocation.CAN.kShooterR.createTalonFX();
        
        TalonFXConfigHelper config = new TalonFXConfigHelper();
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        config.addTunable(Constants.kShooterPID);
        
        config.apply(motorL, motorR);
        targetState = State.IDLE;

        setDefaultCommand(commandSetState(ShooterSubsystem.State.IDLE));
    }

    @Override
    public void periodic() {
        
        double targetRPS = 0;
        switch (targetState) {
            case SHOOT -> targetRPS = AimCalc.getInstance().getShooterRPS(); 
            case IDLE -> targetRPS = Constants.kShooterIdleRPS.get();
            case WARM -> targetRPS = Constants.kShooterWarmRPS.get(); // warm motor and keep it spinning so it can accelerate faster when we want to shoot, but this may need to be tuned based on the actual shooter mechanism and how much warmup is needed. Start with a value around 50-70% of the full shooting speed and adjust as needed based on testing.
            case RINDEX -> targetRPS = -Constants.kShooterRindexRPS.get(); // Run the shooter in reverse at full speed to help unjam balls or index them backwards. This is useful if a ball gets stuck in the shooter or if we want to move balls backwards from the indexer into the shooter. The speed can be adjusted as needed, but starting with full reverse speed is a good way to ensure it can clear jams effectively.
        }



        motorL.setControl(velocityControl.withVelocity(targetRPS).withEnableFOC(true));
        motorR.setControl(velocityControl.withVelocity(targetRPS).withEnableFOC(true));
    }

    public void setTargetState(State targetState) {
        this.targetState = targetState;
    }

    public Command commandSetState(State targetState) {
        return Commands.run(() -> {
            setTargetState(targetState);
        }, this);
    }
}