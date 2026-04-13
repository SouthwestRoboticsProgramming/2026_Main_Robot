package com.swrobotics.robot.config;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.hardware.CANrange;
import edu.wpi.first.wpilibj.RobotBase;

/**
 * Describes the IDs of the various devices within the robot.
 */
public final class IOAllocation {
    public static final class CAN {
        private static final String kRIO = "rio";
        public static final String kGerald = "Gerald";

        // All on Gerald
        public static final String kSwerveBus = kGerald;
        public static final SwerveIDs kSwerveFL = new SwerveIDs(9, 5, 1);
        public static final SwerveIDs kSwerveFR = new SwerveIDs(10, 6, 2);
        public static final SwerveIDs kSwerveBL = new SwerveIDs(11, 7, 3);
        public static final SwerveIDs kSwerveBR = new SwerveIDs(12, 8, 4);
        public static final CanId kJosh = new CanId(13, kGerald); // Gyroscope

        public static final CanId kPDP = new CanId(50, kRIO);

        public static final CanId kShooterL = new CanId(14, kGerald);
        public static final CanId kShooterR = new CanId(15, kGerald);
        public static final CanId kHoodMotor = new CanId(16, kGerald);
        public static final CanId kTurretMotor = new CanId(24, kGerald);
        public static final CanId kTurretCANcoder = new CanId(25, kGerald);

        public static final CanId kIndexerFloor = new CanId(17, kGerald);
        public static final CanId kIndexerShooter = new CanId(18, kGerald);
        public static final CanId kIndexerBelt = new CanId(19, kGerald);
        public static final CanId kIndexerKicker = new CanId(20, kGerald);
        public static final CanId kIndexerCANrange = new CanId(21, kGerald);
        public static final CanId kIntakeMotor = new CanId(22, kGerald);
        public static final CanId kExpansionMotor = new CanId(23, kGerald);
        
    }

    public static final class RIO {
        public static final int kPWM_LEDs = 0;
    }

    /** IDs of the devices within one swerve module */
    public static final class SwerveIDs {
        public final CanId drive, turn, encoder;

        public SwerveIDs(int drive, int turn, int encoder) {
            this.drive = new CanId(drive, CAN.kSwerveBus);
            this.turn = new CanId(turn, CAN.kSwerveBus);
            this.encoder = new CanId(encoder, CAN.kSwerveBus);
        }
    }

    /** Location of a CAN device. This includes both the ID and the CAN bus */
    public static final class CanId {
        public static int uniqueifyForSim(int id, String bus) {
            // CTRE sim doesn't support CANivore, so ids need to be globally unique
            if (RobotBase.isSimulation() && bus.equals(CAN.kGerald)) {
                // If we somehow have more than 32 devices on the RoboRIO bus
                // we have other problems
                return id + 32;
            }

            return id;
        }

        private final int id;
        private final String bus;

        public CanId(int id, String bus) {
            this.id = id;
            this.bus = bus;
        }

        public int id() {
            return uniqueifyForSim(id, bus);
        }

        public String bus() {
            if (RobotBase.isSimulation())
                return CAN.kRIO;

            return bus;
        }

        public TalonFX createTalonFX() {
            return new TalonFX(id(), new CANBus(bus()));
        }

        public TalonFXS createTalonFXS() {
            return new TalonFXS(id(), new CANBus(bus()));
        }

        public CANcoder createCANcoder() {
            return new CANcoder(id(), new CANBus(bus()));
        }
        public CANrange createCANrange() {
            return new CANrange(id(), new CANBus(bus()));
        }
    }
}
