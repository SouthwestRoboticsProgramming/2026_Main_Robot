package com.swrobotics.robot.subsystems.shooter;

// Ctre imports
import com.ctre.phoenix6.controls.VelocityVoltage; // Switched to Velocity
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

// SwRobotics imports
import com.swrobotics.lib.ctre.TalonFXConfigHelper;
import com.swrobotics.robot.config.Constants;
import com.swrobotics.robot.config.IOAllocation;

// WPILib imports
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

// Java imports
import java.util.function.DoubleSupplier;

public class ShooterSubsystem extends SubsystemBase {

    // Define shooter states for easier control
    public enum State {
        IDLE, // Shooter is stopped
        SHOOT, // Shooter is at full speed for shooting
        WARM, // Shooter is at a lower speed to warm up but not shoot
        RINDEX // Shooter is running in reverse to help unjam balls or index them backwards
    }
    
    // Motors
    private final TalonFX motorL;
    private final TalonFX motorR;

    // Control mode for velocity control
    private final VelocityVoltage velocityControl = new VelocityVoltage(0);
    private State targetState;

    // Multiplier for variable trigger shooting (0.0 to 1.0)
    private double speedMultiplier = 1.0; 

    public ShooterSubsystem() {

        // Initialize motors using IOAllocation for better organization and to avoid hardcoding CAN IDs
        motorL = IOAllocation.CAN.kShooterL.createTalonFX();
        motorR = IOAllocation.CAN.kShooterR.createTalonFX();
        
        // Configure motors using a helper class to keep this constructor clean and organized
        TalonFXConfigHelper config = new TalonFXConfigHelper();
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        // --- PID GAINS ---
        //TODO: These values need tuning! Start with kP at 0.1 and kV at 0.12 DO NOT SET kI or kD YET, as they are not always necessary and can cause instability if set too high
        config.Slot0.kP = 0.32; // kP is the primary gain for velocity control, start with a small value and increase until you see good response without oscillation
        config.Slot0.kI = 0.0; // DO NOT TOUCH kI!
        config.Slot0.kD = 0.001; // kD can help reduce overshoot and improve stability, but start with a very small value and increase if you see oscillation or overshoot
        config.Slot0.kV = 0.13; // kV is vital for velocity control 
        config.Slot0.kA = 0.53; // kA is not always necessary, but can help improve response at higher speeds. 

        // Apply the configuration to both motors
        config.apply(motorL, motorR);
        targetState = State.IDLE;
    }

    @Override
    public void periodic() {
        
        // Determine target RPS based on the current target state
        double targetRPS = 0;
        switch (targetState) {

            case SHOOT -> targetRPS = Constants.kShooterRPS.get() * speedMultiplier; // Shooter speed is positive to shoot out, but this may need to be reversed based on the actual motor orientation on the robot. If the shooter runs backwards, simply change this to negative or set to counterclockwise. (Scales with trigger pull percentage)
            case IDLE -> targetRPS = Constants.kShooterIdleRPS.get(); // Idle speed is 0
            case WARM -> targetRPS = Constants.kShooterWarmRPS.get(); // warm motor and keep it spinning so it can accelerate faster when we want to shoot, but this may need to be tuned based on the actual shooter mechanism and how much warmup is needed. Start with a value around 50-70% of the full shooting speed and adjust as needed based on testing.
            case RINDEX -> targetRPS = -Constants.kShooterRPS.get(); // Run the shooter in reverse at full speed to help unjam balls or index them backwards. This is useful if a ball gets stuck in the shooter or if we want to move balls backwards from the indexer into the shooter. The speed can be adjusted as needed, but starting with full reverse speed is a good way to ensure it can clear jams effectively.
        }

        // Apply control
        motorL.setControl(velocityControl.withVelocity(targetRPS));
        motorR.setControl(velocityControl.withVelocity(targetRPS));
    }

    // Method to change the target state of the shooter.
    public void setTargetState(State targetState) {
        this.targetState = targetState;
    }

    // Method to update the speed multiplier based on trigger pull
    public void setSpeedMultiplier(double multiplier) {
        // Clamp the value between 0.0 and 1.0 to prevent unexpected behavior
        this.speedMultiplier = Math.max(0.0, Math.min(1.0, multiplier));
    }

    // Command to set State of the Shooter.
    public Command commandSetState(State targetState) {
        return Commands.run(() -> {
            setTargetState(targetState);
            setSpeedMultiplier(1.0); // Reset multiplier to 100% for standard button presses
        }, this);
    }
}