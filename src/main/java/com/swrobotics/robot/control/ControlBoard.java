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

import com.swrobotics.robot.subsystems.intake.indexer.IndexerSubsystem;
import com.swrobotics.robot.subsystems.intake.IntakeSubsystem;
import com.swrobotics.robot.subsystems.intake.expansion.ExpansionSubsystem;
import com.swrobotics.robot.subsystems.intake.expansion.ExpansionSubsystem.State;
import com.swrobotics.robot.subsystems.shooter.ShooterSubsystem;
//import com.swrobotics.robot.subsystems.shooter.hood.HoodSubsystem;
import com.swrobotics.robot.subsystems.shooter.hood.HoodSubsystem.HoodState;

public final class ControlBoard extends SubsystemBase {
    /*
     * Control mapping:
     *
     * Driver:
     * Left stick: drive translation
     * Right stick X: drive rotation
     * 
     * Left trigger: intake
     * Left bumper: reverse indexer
     * Right trigger: shoot on move
     * 
     * X Button: expansion
     * y Button: Pass mode
     * a Button: log successful shot (for tuning)
     * b Button: hood homing
     * 
     * D-Pad Up: reset gyro
     * D-Pad Down: Brake mode (lock swerve modules)
     *
     * Operator:
     * Right trigger: Pass 
     * Left trigger: Shoot
     * 
     * X Button: manual hood nudge up
     * B Button: manual hood nudge down
     * 
     * D-Pad Up: hood homing
     */

    
    private static final NTEntry<Boolean> CHARACTERISE_WHEEL_RADIUS = new NTBoolean("Drive/Characterize Wheel Radius", false);

    private final RobotContainer robot;
    public final CommandXboxController driver;
    public final CommandXboxController operator;
    public final CommandXboxController testController;


    private final DriveAccelFilter driveControlFilter;

    public ControlBoard(RobotContainer robot) {
        this.robot = robot;

        driver = new CommandXboxController(Constants.kDriverControllerPort);
        operator = new CommandXboxController(Constants.kOperatorControllerPort);
        testController = new CommandXboxController(2);

        driveControlFilter = new DriveAccelFilter(Constants.kDriveControlMaxAccel);

        configureControls();
        configureRumbles();
    }

    private void configureControls() {
        // Gyro reset buttons
        
        driver.back().onFalse(Commands.runOnce(() -> robot.drive.resetRotation(new Rotation2d()))); // Two buttons to reset gyro so the driver can't get confused

        robot.drive.setDefaultCommand(DriveCommands.driveFieldRelative(
                robot.drive,
                () -> this.getDriveTranslation(),
                this::getDriveRotation
        ));

        new Trigger(CHARACTERISE_WHEEL_RADIUS::get).whileTrue(new CharacterizeWheelsCommand(robot.drive));
        
        /* --- Intake/indexer --- */

        driver.leftTrigger().whileTrue(robot.intake.commandSetState(IntakeSubsystem.State.INTAKE));

        driver.leftBumper().whileTrue(robot.indexer.commandSetState(IndexerSubsystem.State.RINDEX));

        driver.rightTrigger().whileTrue(robot.hood.setMode(HoodState.AUTO_TRACK)
                .alongWith(robot.shooter.commandSetState(ShooterSubsystem.State.AUTO)
                .withTimeout(1)
                .andThen(robot.indexer.commandSetState(IndexerSubsystem.State.FEED))
                .alongWith(robot.expansion.commandSetState(ExpansionSubsystem.State.SHOOT))));
                
        driver.rightBumper()
                .whileTrue(robot.shooter.commandSetState(ShooterSubsystem.State.PASS)
                .alongWith(robot.hood.setMode(HoodState.PASSING))
                .alongWith(robot.indexer.commandSetState(IndexerSubsystem.State.RINDEX))
                .withTimeout(1.5)
                .andThen(robot.indexer.commandSetState(IndexerSubsystem.State.FEED)));

        // driver.rightTrigger().whileTrue(DriveCommands.shootOnTheMove(robot.drive, robot.shooter, robot.hood, robot.indexer,
        //                 () -> -driver.getLeftY(),
        //                 () -> -driver.getLeftX()
        //         ));

        driver.x().toggleOnTrue(robot.expansion.commandSetState(ExpansionSubsystem.State.EXTENDED));
        // driver.b().whileTrue(robot.hood.setMode(HoodState.HOMING));
        // driver.y().whileTrue(robot.hood.setMode(HoodState.PASSING));
        // driver.a().onTrue(Telemetry.logSuccessfulShot());
        // driver.y().onTrue(robot.hood.manualNudge(1.0)).onFalse(Commands.runOnce(() -> robot.hood.setMode(HoodState.IDLE)));
        // driver.b().onTrue(robot.hood.manualNudge(-1.0)).onFalse(Commands.runOnce(() -> robot.hood.setMode(HoodState.IDLE)));
        driver.povUp().onFalse(Commands.runOnce(() -> robot.drive.resetRotation(new Rotation2d())));
        driver.povDown().toggleOnTrue(Commands.runOnce(() -> robot.drive.commandLockModules()));

        /* --- MANUAl OVERRIDES --- */       
        operator.rightTrigger()
                .whileTrue(robot.shooter.commandSetState(ShooterSubsystem.State.PASS)
                .alongWith(robot.hood.setMode(HoodState.PASSING))
                .withTimeout(1.5)
                .andThen(robot.indexer.commandSetState(IndexerSubsystem.State.FEED)));

        operator.leftTrigger()
                .whileTrue(robot.shooter.commandSetState(ShooterSubsystem.State.SHOOT)
                .withTimeout(.75)
                .andThen(robot.indexer.commandSetState(IndexerSubsystem.State.FEED)));

        operator.x().onTrue(robot.hood.manualNudge(3.0));
        operator.b().onTrue(robot.hood.manualNudge(-3.0));
        operator.povUp()
                .whileTrue(robot.hood.setMode(HoodState.HOMING));

        //testController.leftTrigger().whileTrue(robot.intake.commandSetState(IntakeSubsystem.State.INTAKE).alongWith(robot.indexer.commandSetState(IndexerSubsystem.State.INTAKE)));
        //testController.leftBumper().whileTrue(robot.indexer.commandSetState(IndexerSubsystem.State.RINDEX));
        testController.rightTrigger().whileTrue(robot.hood.setMode(HoodState.AUTO_TRACK).alongWith(robot.shooter.commandSetState(ShooterSubsystem.State.AUTO)).alongWith(robot.expansion.commandSetState(State.SHOOT)));
        testController.rightBumper().whileTrue(robot.hood.setMode(HoodState.PASSING).alongWith(robot.shooter.commandSetState(ShooterSubsystem.State.PASS)).alongWith(robot.expansion.commandSetState(State.SHOOT)));
        
        //testController.y().whileTrue(DriveCommands.shootOnTheMove(robot.drive, robot.shooter, robot.hood, robot.indexer,() -> -driver.getLeftY(), () -> -driver.getLeftX()));
        //testController.x().toggleOnTrue(robot.expansion.commandSetState(ExpansionSubsystem.State.EXTENDED));
        //testController.b().toggleOnTrue(Commands.runOnce(() -> robot.drive.commandLockModules()));       
        
        
        testController.povUp().onTrue(robot.hood.manualNudge(1.0));
        testController.povLeft().whileTrue(robot.hood.setMode(HoodState.HOMING));
        testController.povDown().onTrue(robot.hood.manualNudge(-1.0));
        //testController.povRight().onFalse(Commands.runOnce(() -> robot.drive.resetRotation(new Rotation2d())));
}

    /**
     * @return translation input for the drive base, in meters/sec
     */
    private Translation2d getDriveTranslation() {
        double maxSpeed = Constants.kDriveMaxAchievableSpeed;

        Translation2d leftStick = MathUtil.deadband2d(
            new Translation2d(driver.getLeftX() , driver.getLeftY()),
            Constants.kDeadband
        );


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
                            && DriverStation.getMatchTime() <= Constants.kActive_InactiveAlert1Time2)
            .onTrue(RumblePatternCommands.inactive_Active_TransferAlert(driver, 0.75)
                    .alongWith(RumblePatternCommands.inactive_Active_TransferAlert(operator, 0.75)));


        
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
                            && DriverStation.getMatchTime() <= Constants.kActive_InactiveAlert3Time2)
            .onTrue(RumblePatternCommands.inactive_Active_TransferAlert(driver, 0.75)
                    .alongWith(RumblePatternCommands.inactive_Active_TransferAlert(operator, 0.75)));
        


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
    }
}
