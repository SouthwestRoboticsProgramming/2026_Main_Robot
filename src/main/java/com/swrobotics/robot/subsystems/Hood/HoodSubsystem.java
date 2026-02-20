package com.swrobotics.robot.subsystems.Hood;

import com.swrobotics.robot.config.Constants;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class HoodSubsystem extends SubsystemBase {
    public enum State {
        GO_TO_START,
        AUTO_TRACKING,
        MANUAL_MODE
    }

    private final HoodIO io;
    private final HoodIOInputsAutoLogged inputs = new HoodIOInputsAutoLogged();

    private State targetState = State.GO_TO_START;
    private double manualTargetRotations = 0.0;
    private double currentHoodAngle = 0.0;
    private Pose2d robotPose;
    private Rotation2d hubAngle;
    private double shooterTargetRPS = Constants.kShooterRPS.get();
    private double computedAngleFromRegression = 0.0;

    public HoodSubsystem(HoodIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Hood", inputs);

        currentHoodAngle = inputs.positionDeg;

        double maxAngle = Constants.kHoodMaxAngle.get();
        double minAngle = Constants.kHoodMinAngle.get();
        double maxRotations = maxAngle / 360.0;
        double minRotations = minAngle / 360.0;

        switch (targetState) {
            case GO_TO_START:
                manualTargetRotations = 0.0;
                break;

            case AUTO_TRACKING:
                if (robotPose != null) {
                    Pose2d hubPose = Constants.kHubPose;
                    double dx = hubPose.getX() - robotPose.getX();
                    double dy = hubPose.getY() - robotPose.getY();
                    double distance = Math.hypot(dx, dy);

                    hubAngle = new Rotation2d(Math.atan2(dy, dx));

                    double a0 = Constants.kA0.get();
                    double a1 = Constants.kA1.get();
                    double a2 = Constants.kA2.get();
                    double aRps = Constants.kA_Rps.get();

                    double desiredAngleDeg =
                            a0 + a1 * distance + a2 * distance * distance + aRps * shooterTargetRPS;

                    computedAngleFromRegression = desiredAngleDeg;

                    desiredAngleDeg = Math.max(minAngle, Math.min(maxAngle, desiredAngleDeg));

                    double desiredRotations = desiredAngleDeg / 360.0;
                    manualTargetRotations = Math.max(minRotations, Math.min(maxRotations, desiredRotations));
                }
                break;

            case MANUAL_MODE:
                manualTargetRotations = Math.max(minRotations, Math.min(maxRotations, manualTargetRotations));
                break;
        }

        io.setPosition(manualTargetRotations);

        Logger.recordOutput("Hood/State", targetState.name());
        Logger.recordOutput("Hood/TargetAngleDeg", manualTargetRotations * 360.0);
        Logger.recordOutput("Hood/ComputedAngleFromRegression", computedAngleFromRegression);
        Logger.recordOutput("Hood/CurrentAngleDeg", currentHoodAngle);
    }

    public void setTargetState(State state) {
        this.targetState = state;
    }

    public State getCurrentState() {
        return targetState;
    }

    public double getCurrentAngleDegrees() {
        return currentHoodAngle;
    }

    public Rotation2d getHubAngle() {
        return hubAngle;
    }

    public void setRobotPose(Pose2d pose) {
        this.robotPose = pose;
    }

    public void setShooterTargetRPS(double rps) {
        this.shooterTargetRPS = rps;
    }

    public void setManualPosition(double rotations) {
        this.manualTargetRotations = rotations;
    }

    public Command commandSetState(State state) {
        return Commands.runOnce(() -> setTargetState(state), this);
    }

    public Command commandHome() {
        if (targetState == State.AUTO_TRACKING) {
            return Commands.none();
        } else {
            return Commands.runOnce(() -> setTargetState(State.GO_TO_START), this);
        }
    }

    public Command commandToggleAutoManual() {
        return Commands.runOnce(() -> {
            if (targetState == State.AUTO_TRACKING) {
                setTargetState(State.MANUAL_MODE);
            } else {
                setTargetState(State.AUTO_TRACKING);
            }
        }, this);
    }

    public Command commandManualDown() {
        if (targetState != State.MANUAL_MODE) {
            return Commands.none();
        } else if (getCurrentAngleDegrees() <= Constants.kHoodMinAngle.get()) {
            return Commands.none();
        } else {
            return Commands.runOnce(() -> {
                double newAngle = getCurrentAngleDegrees() - 5.0;
                double newPos = Math.max(
                        Constants.kHoodMinAngle.get() / 360.0,
                        newAngle / 360.0);
                setManualPosition(newPos);
            }, this);
        }
    }

    public Command commandManualUp() {
        if (targetState != State.MANUAL_MODE) {
            return Commands.none();
        } else if (getCurrentAngleDegrees() >= Constants.kHoodMaxAngle.get()) {
            return Commands.none();
        } else {
            return Commands.runOnce(() -> {
                double newAngle = getCurrentAngleDegrees() + 5.0;
                double newPos = Math.min(
                        Constants.kHoodMaxAngle.get() / 360.0,
                        newAngle / 360.0);
                setManualPosition(newPos);
            }, this);
        }
    }

    public Command commandManualJog(double degrees) {
        return Commands.runOnce(() -> {
            setTargetState(State.MANUAL_MODE);
            double newAngle = getCurrentAngleDegrees() + degrees;
            double minAngle = Constants.kHoodMinAngle.get();
            double maxAngle = Constants.kHoodMaxAngle.get();
            newAngle = Math.max(minAngle, Math.min(maxAngle, newAngle));
            double newPos = newAngle / 360.0;
            setManualPosition(newPos);
        }, this);
    }
}
