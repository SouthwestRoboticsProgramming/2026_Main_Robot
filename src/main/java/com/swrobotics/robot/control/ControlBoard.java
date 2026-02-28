package com.swrobotics.robot.control;

import com.swrobotics.lib.field.FieldInfo;
import com.swrobotics.lib.net.NTBoolean;
import com.swrobotics.lib.net.NTEntry;
import com.swrobotics.lib.utils.MathUtil;
import com.swrobotics.robot.RobotContainer;
import com.swrobotics.robot.commands.CharacterizeWheelsCommand;
import com.swrobotics.robot.commands.DriveCommands;
import com.swrobotics.robot.commands.RumblePatternCommands;
import com.swrobotics.robot.config.Constants;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import com.swrobotics.robot.subsystems.climber.ClimberSubsystem;
import com.swrobotics.robot.subsystems.intake.expansions.ExpansionSubsystem;
import com.swrobotics.robot.subsystems.intake.indexer.IndexerSubsystem;
import com.swrobotics.robot.subsystems.intake.IntakeSubsystem;
import com.swrobotics.robot.subsystems.shooter.ShooterSubsystem;
import com.swrobotics.robot.subsystems.shooter.hood.HoodSubsystem;

public final class ControlBoard extends SubsystemBase {
    /*
     * Control mapping:
     *
     * Driver:
     * Left stick: drive translation
     * Right stick X: drive rotation
     * Right trigger: shoot
     * Left trigger: intake
     * Right bumber: feed
     * left bumber: rindex
     * 
     * Start: Reset gyro
     * Back: Reset gyro
     * 
     * Operator:
     * 
     */

    private static final NTEntry<Boolean> CHARACTERISE_WHEEL_RADIUS = new NTBoolean("Drive/Characterize Wheel Radius", false);

    private final RobotContainer robot;
    public final CommandXboxController driver;
    public final CommandXboxController operator;

    private final DriveAccelFilter driveControlFilter;

    public ControlBoard(RobotContainer robot) {
        this.robot = robot;

        driver = new CommandXboxController(Constants.kDriverControllerPort);
        operator = new CommandXboxController(Constants.kOperatorControllerPort);

        driveControlFilter = new DriveAccelFilter(Constants.kDriveControlMaxAccel);

        configureControls();
        configureRumbles();
    }

    private void configureControls() {
        // Gyro reset buttons
        driver.start().onFalse(Commands.run(() -> robot.drive.resetRotation(new Rotation2d())));
        driver.back().onFalse(Commands.run(() -> robot.drive.resetRotation(new Rotation2d()))); // Two buttons to reset gyro so the driver can't get confused

        robot.drive.setDefaultCommand(DriveCommands.driveFieldRelative(
                robot.drive,
                () -> this.getDriveTranslation(),
                this::getDriveRotation
        ));

        new Trigger(CHARACTERISE_WHEEL_RADIUS::get).whileTrue(new CharacterizeWheelsCommand(robot.drive));
        
        /* --- Intake/indexer --- */

        driver.leftTrigger()
                .whileTrue(robot.indexer.commandSetState(IndexerSubsystem.State.INTAKE)
                .alongWith(robot.intake.commandSetState(IntakeSubsystem.State.INTAKE)));
        driver.rightBumper()
                .whileTrue(robot.indexer.ConditionalIntake()
                .alongWith(robot.intake.commandSetState(IntakeSubsystem.State.INTAKE)));
        driver.leftBumper()
                .whileTrue(robot.indexer.commandSetState(IndexerSubsystem.State.RINDEX)
                .alongWith(robot.shooter.commandSetState(ShooterSubsystem.State.RINDEX)));

        /* --- Shooter --- */
        driver.rightTrigger()
        .whileTrue(robot.shooter.commandSetState(ShooterSubsystem.State.SHOOT));

        /* --- Hood --- */
        // operator.povDown().onTrue(robot.hood.commandManualUp());
        // operator.povUp().onTrue(robot.hood.commandManualDown());

        /* --- expansion --- */
        operator.x().whileTrue(robot.expansion.commandSetState(ExpansionSubsystem.State.EXTENDED));

        /* --- Climber --- */
        operator.y().toggleOnTrue(robot.climber.commandSetState(ClimberSubsystem.State.EXTENDED));

        driver.a()
                .toggleOnTrue(DriveCommands.driveFieldRelativeSnapToHub(robot.drive, () -> this.getDriveTranslation())
                .alongWith(robot.hood.setMode(HoodSubsystem.HoodMode.AUTO_TRACK)))
                .onFalse(robot.hood.setMode(HoodSubsystem.HoodMode.MANUAL));

        driver.povDown().whileTrue(DriveCommands.driveThroughBump(robot.drive));
        driver.povUp().whileTrue(DriveCommands.driveThroughTrench(robot.drive));

    }

    /**
     * @return translation input for the drive base, in meters/sec
     */
    private Translation2d getDriveTranslation() {
        double maxSpeed = Constants.kDriveMaxAchievableSpeed;

        Translation2d leftStick = MathUtil.deadband2d(
            new Translation2d(driver.getLeftX(), driver.getLeftY()),
            Constants.kDeadband
        );

        // Apply an exponential curve to the driver's input. This allows the
        // driver to have slower, more precise movement in the center of the
        // stick, while still having high speed movement towards the edges.
        double rawMag = leftStick.getNorm();
        double powerMag = MathUtil.powerWithSign(rawMag, Constants.kDriveControlDrivePower);

        // Prevent division by zero, which would result in a target velocity of
        // (NaN, NaN), which motor controllers do not like
        if (rawMag == 0 || powerMag == 0)
            return new Translation2d(0, 0);

        double targetSpeed = powerMag * maxSpeed;
        double filteredSpeed = driveControlFilter.calculate(targetSpeed);
        return new Translation2d(-leftStick.getY(), -leftStick.getX())
            .div(rawMag) // Normalize translation
            .times(filteredSpeed) // Apply new speed
            .rotateBy(FieldInfo.getAllianceForwardAngle()); // Account for driver's perspective
    }

    /**
     * @return radians per second input for the drive base
     */
    private double getDriveRotation() {
        double rightStickX = MathUtil.deadband(driver.getRightX(), Constants.kDeadband);
        double input = MathUtil.powerWithSign(-rightStickX, Constants.kDriveControlTurnPower);
        return Units.rotationsToRadians(input * Constants.kDriveControlMaxTurnSpeed);
    }

    private void configureRumbles() {
        // Transfer rumble
        new Trigger(
            () ->
                    DriverStation.isTeleopEnabled()
                            && DriverStation.getMatchTime() > 0
                            && DriverStation.getMatchTime() <= Constants.kTransferAlertTime)
            .onTrue(RumblePatternCommands.inactive_Active_TransferAlert(driver, 0.75)
                    .alongWith(RumblePatternCommands.inactive_Active_TransferAlert(operator, 0.75)));

        new Trigger(
                () ->
                        DriverStation.isTeleopEnabled()
                                && DriverStation.getMatchTime() > 0
                                && DriverStation.getMatchTime() <= Constants.kActive_InactiveAlert1Time)
                .onTrue(RumblePatternCommands.inactive_Active_TransferAlert(driver, 0.5)
                        .alongWith(RumblePatternCommands.inactive_Active_TransferAlert(operator, 0.5)));

        new Trigger(
            () ->
                    DriverStation.isTeleopEnabled()
                            && DriverStation.getMatchTime() > 0
                            && DriverStation.getMatchTime() <= Constants.kActive_InactiveAlert1Time2)
            .onTrue(RumblePatternCommands.inactive_Active_TransferAlert(driver, 0.75)
                    .alongWith(RumblePatternCommands.inactive_Active_TransferAlert(operator, 0.75)));


        new Trigger(
            () ->
                    DriverStation.isTeleopEnabled()
                            && DriverStation.getMatchTime() > 0
                            && DriverStation.getMatchTime() <= Constants.kActive_InactiveAlert2Time)
            .onTrue(RumblePatternCommands.inactive_Active_TransferAlert(driver, 0.5)
                    .alongWith(RumblePatternCommands.inactive_Active_TransferAlert(operator, 0.5)));
        
        new Trigger(
            () ->
                    DriverStation.isTeleopEnabled()
                            && DriverStation.getMatchTime() > 0
                            && DriverStation.getMatchTime() <= Constants.kActive_InactiveAlert2Time2)
            .onTrue(RumblePatternCommands.inactive_Active_TransferAlert(driver, 0.75)
                    .alongWith(RumblePatternCommands.inactive_Active_TransferAlert(operator, 0.75)));
        
        new Trigger(
            () ->
                    DriverStation.isTeleopEnabled()
                            && DriverStation.getMatchTime() > 0
                            && DriverStation.getMatchTime() <= Constants.kActive_InactiveAlert3Time)
            .onTrue(RumblePatternCommands.inactive_Active_TransferAlert(driver, 0.5)
                    .alongWith(RumblePatternCommands.inactive_Active_TransferAlert(operator, 0.5)));
        
        new Trigger(
            () ->
                    DriverStation.isTeleopEnabled()
                            && DriverStation.getMatchTime() > 0
                            && DriverStation.getMatchTime() <= Constants.kActive_InactiveAlert3Time2)
            .onTrue(RumblePatternCommands.inactive_Active_TransferAlert(driver, 0.75)
                    .alongWith(RumblePatternCommands.inactive_Active_TransferAlert(operator, 0.75)));
        
        new Trigger(
            () ->
                    DriverStation.isTeleopEnabled()
                            && DriverStation.getMatchTime() > 0
                            && DriverStation.getMatchTime() <= Constants.kActive_InactiveAlert4Time)
            .onTrue(RumblePatternCommands.inactive_Active_TransferAlert(driver, 0.5)
                    .alongWith(RumblePatternCommands.inactive_Active_TransferAlert(operator, 0.5)));

        new Trigger(
            () ->
                    DriverStation.isTeleopEnabled()
                            && DriverStation.getMatchTime() > 0
                            && DriverStation.getMatchTime() <= Constants.kActive_InactiveAlert4Time2)
            .onTrue(RumblePatternCommands.inactive_Active_TransferAlert(driver, 0.75)
                    .alongWith(RumblePatternCommands.inactive_Active_TransferAlert(operator, 0.75)));
                    
                    
        // Endgame Notice (controller rumble)
        new Trigger(
                () ->
                        DriverStation.isTeleopEnabled()
                                && DriverStation.getMatchTime() > 0
                                && DriverStation.getMatchTime() <= Constants.kEndgameAlertTime)
                .onTrue(RumblePatternCommands.endgameAlert(driver, 0.75)
                        .alongWith(RumblePatternCommands.endgameAlert(operator, 0.75)));

        new Trigger(
                 () ->
                        DriverStation.isTeleopEnabled()
                                && DriverStation.getMatchTime() > 0
                                && DriverStation.getMatchTime() <= Constants.kEndgameAlert2Time)
                .onTrue(RumblePatternCommands.endgameAlertFinalCountdown(driver, 0.75));
    }

    @Override
    public void periodic() {
        AimCalc.getInstance().setSpeedMultiplier(driver.getRightTriggerAxis());
    }
}
