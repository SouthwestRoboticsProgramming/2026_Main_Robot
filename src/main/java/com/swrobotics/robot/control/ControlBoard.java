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
import com.swrobotics.robot.config.FieldPositions;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import com.swrobotics.robot.subsystems.intake.intakeArm.IntakeArmSubsystem;
import com.swrobotics.robot.subsystems.indexer.IndexerSubsystem;
import com.swrobotics.robot.subsystems.intake.IntakeSubsystem;
import com.swrobotics.robot.subsystems.shooter.ShooterSubsystem;
import com.swrobotics.robot.subsystems.swerve.SwerveDriveSubsystem;

public final class ControlBoard extends SubsystemBase {

    /*
     *
     * Control mapping:
     *
     * Driver:
     * Left stick: drive translation
     * Right stick X: Drive rotation
     * Left trigger: Intake
     * Left bumper: Outake
     * Right trigger: Shoot
     * Right bumper: pass
     * Y: Extend intake arm
     * X: Indexer feed
     * A: Aim at hub
     * POV left/right: Aim pass left or right
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

        driver.rightTrigger().whileTrue(robot.shooter.commandSetState(ShooterSubsystem.State.AUTO));
        //.alongWith(robot.intakeArm.commandShoot()));

        driver.rightBumper().whileTrue(robot.shooter.commandSetState(ShooterSubsystem.State.PASS)
        .alongWith(robot.intakeArm.commandShoot()));     
        driver.x().toggleOnTrue(robot.indexer.commandSetState(IndexerSubsystem.State.FEED));
        driver.b().whileTrue(DriveCommands.driveSnapToHub(robot.drive, () -> -driver.getLeftY(), () -> -driver.getLeftX()));
        driver.povRight().whileTrue(DriveCommands.driveSnapToLeftCorner(robot.drive, () -> -driver.getLeftY(), () -> -driver.getLeftX()));
        driver.povLeft().whileTrue(DriveCommands.driveSnapToRightCorner(robot.drive, () -> -driver.getLeftY(), () -> -driver.getLeftX()));
        

        



        driver.y().toggleOnTrue(robot.intakeArm.commandSetState(IntakeArmSubsystem.State.EXTENDED));
        driver.povUp().onFalse(Commands.runOnce(() -> robot.drive.resetRotation(new Rotation2d())));

        /* --- MANUAl OVERRIDES --- */       
        operator.rightTrigger()
                .whileTrue(robot.shooter.commandSetState(ShooterSubsystem.State.PASS)
                .withTimeout(.75)
                .andThen(robot.indexer.commandSetState(IndexerSubsystem.State.FEED)));

        operator.rightTrigger()
                .whileTrue(robot.shooter.commandSetState(ShooterSubsystem.State.SHOOT)
                .withTimeout(.75)
                .andThen(robot.indexer.commandSetState(IndexerSubsystem.State.FEED)));

     
        
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
                            && DriverStation.getMatchTime() <= FieldPositions.kTransferAlertTime)
            .onTrue(RumblePatternCommands.inactive_Active_TransferAlert(driver, 0.75)
                    .alongWith(RumblePatternCommands.inactive_Active_TransferAlert(operator, 0.75)));



        new Trigger(
            () ->
                    DriverStation.isTeleopEnabled()
                            && DriverStation.getMatchTime() > 0
                            && DriverStation.getMatchTime() <= FieldPositions.kActive_InactiveAlert1Time2)
            .onTrue(RumblePatternCommands.inactive_Active_TransferAlert(driver, 0.75)
                    .alongWith(RumblePatternCommands.inactive_Active_TransferAlert(operator, 0.75)));


        
        new Trigger(
            () ->
                    DriverStation.isTeleopEnabled()
                            && DriverStation.getMatchTime() > 0
                            && DriverStation.getMatchTime() <= FieldPositions.kActive_InactiveAlert2Time2)
            .onTrue(RumblePatternCommands.inactive_Active_TransferAlert(driver, 0.75)
                    .alongWith(RumblePatternCommands.inactive_Active_TransferAlert(operator, 0.75)));

        
        new Trigger(
            () ->
                    DriverStation.isTeleopEnabled()
                            && DriverStation.getMatchTime() > 0
                            && DriverStation.getMatchTime() <= FieldPositions.kActive_InactiveAlert3Time2)
            .onTrue(RumblePatternCommands.inactive_Active_TransferAlert(driver, 0.75)
                    .alongWith(RumblePatternCommands.inactive_Active_TransferAlert(operator, 0.75)));
        


        new Trigger(
            () ->
                    DriverStation.isTeleopEnabled()
                            && DriverStation.getMatchTime() > 0
                            && DriverStation.getMatchTime() <= FieldPositions.kActive_InactiveAlert4Time2)
            .onTrue(RumblePatternCommands.inactive_Active_TransferAlert(driver, 0.75)
                    .alongWith(RumblePatternCommands.inactive_Active_TransferAlert(operator, 0.75)));
                    
                    
        // Endgame Notice (controller rumble)
        new Trigger(
                () ->
                        DriverStation.isTeleopEnabled()
                                && DriverStation.getMatchTime() > 0
                                && DriverStation.getMatchTime() <= FieldPositions.kEndgameAlertTime)
                .onTrue(RumblePatternCommands.endgameAlert(driver, 0.75)
                        .alongWith(RumblePatternCommands.endgameAlert(operator, 0.75)));

        new Trigger(
                 () ->
                        DriverStation.isTeleopEnabled()
                                && DriverStation.getMatchTime() > 0
                                && DriverStation.getMatchTime() <= FieldPositions.kEndgameAlert2Time)
                .onTrue(RumblePatternCommands.endgameAlertFinalCountdown(driver, 0.75));
    }

    @Override
    public void periodic() {

    }

}
