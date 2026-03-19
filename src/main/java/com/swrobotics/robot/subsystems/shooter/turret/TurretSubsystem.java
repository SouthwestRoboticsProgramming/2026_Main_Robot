package com.swrobotics.robot.subsystems.shooter.turret;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.swrobotics.lib.ctre.TalonFXConfigHelper;
import com.swrobotics.robot.config.IOAllocation;
import com.swrobotics.robot.control.AimCalc;
import com.swrobotics.robot.subsystems.swerve.SwerveDriveSubsystem;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public final class TurretSubsystem extends SubsystemBase {
    public enum TurretMode { SCORING, IDLE }

    private static final double TURRET_GEAR_TEETH = 200.0;
    private static final double MOTOR_GEAR_TEETH = 40.0;
    private static final double MOTOR_TO_TURRET_RATIO = TURRET_GEAR_TEETH / MOTOR_GEAR_TEETH;

    private static final double IDLE_POS_ROT = 270.0 / 360.0;
    private static final double TURRET_MIN_ROT = (270.0 - 165.0) / 360.0;
    private static final double TURRET_MAX_ROT = (270.0 + 165.0) / 360.0;
    private static final double ON_TARGET_TOLERANCE_ROT = 1.0 / 360.0;

    private final TalonFX motor;
    private final SwerveDriveSubsystem drive;
    private final MotionMagicVoltage positionControl = new MotionMagicVoltage(0.0).withEnableFOC(true);
    private TurretMode currentMode = TurretMode.IDLE;

    public TurretSubsystem(SwerveDriveSubsystem drive) {
        this.drive = drive;
        motor = IOAllocation.CAN.kTurretMotor.createTalonFX();

        TalonFXConfigHelper config = new TalonFXConfigHelper();
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.Slot0.kP = 18.0; // Snappier turret response
        config.Slot0.kD = 0.3;
        config.MotionMagic.MotionMagicCruiseVelocity = 15.0;
        config.MotionMagic.MotionMagicAcceleration = 40.0;

        config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = TURRET_MAX_ROT * MOTOR_TO_TURRET_RATIO;
        config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = TURRET_MIN_ROT * MOTOR_TO_TURRET_RATIO;

        config.apply(motor);
    }

    @Override
    public void periodic() {
        if (currentMode == TurretMode.SCORING) {
            setControlWithWrap(AimCalc.getInstance().getTurretAimAngle());
        } else {
            setControlWithWrap(Rotation2d.fromRotations(IDLE_POS_ROT));
        }
    }

    private void setControlWithWrap(Rotation2d targetAngle) {
        double currentTurretRot = motor.getPosition().getValueAsDouble() / MOTOR_TO_TURRET_RATIO;
        double error = MathUtil.inputModulus(targetAngle.getRotations() - currentTurretRot, -0.5, 0.5);
        double targetTurretRot = currentTurretRot + error;

        double finalTarget = MathUtil.clamp(targetTurretRot, TURRET_MIN_ROT, TURRET_MAX_ROT);
        
        // Feedforward to compensate for robot spinning
        double chassisYawRateRotPerSec = drive.getRobotRelativeSpeeds().omegaRadiansPerSecond / (2.0 * Math.PI);

        motor.setControl(positionControl
            .withPosition(finalTarget * MOTOR_TO_TURRET_RATIO)
            .withFeedForward(-chassisYawRateRotPerSec * MOTOR_TO_TURRET_RATIO));
    }

    public Command cmdScoring() { return runOnce(() -> currentMode = TurretMode.SCORING); }
    public Command cmdIdle() { return runOnce(() -> currentMode = TurretMode.IDLE); }

    public boolean isOnTarget() {
        double currentTurretRot = motor.getPosition().getValueAsDouble() / MOTOR_TO_TURRET_RATIO;
        double error = MathUtil.inputModulus(AimCalc.getInstance().getTurretAimAngle().getRotations() - currentTurretRot, -0.5, 0.5);
        return Math.abs(error) < ON_TARGET_TOLERANCE_ROT;
    }
}