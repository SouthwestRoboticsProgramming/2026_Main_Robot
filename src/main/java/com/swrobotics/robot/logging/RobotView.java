package com.swrobotics.robot.logging;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import com.swrobotics.robot.subsystems.shooter.hood.HoodSubsystem;

public class RobotView extends TimedRobot {
    // Visualization objects
    private final Mechanism2d m_mech = new Mechanism2d(60, 60);
    private final MechanismRoot2d m_root = m_mech.getRoot("HoodRoot", 30, 20);
    
    // The "Base" of the shooter (static)
    private final MechanismLigament2d m_base = m_root.append(
        new MechanismLigament2d("ShooterBase", 20, 0, 6, new Color8Bit(Color.kGray)));
    
    // The "Hood" that actually moves
    private final MechanismLigament2d m_hoodVisual = m_root.append(
        new MechanismLigament2d("ShooterHood", 25, 26, 10, new Color8Bit(Color.kOrange)));
    
    private final HoodSubsystem hood = new HoodSubsystem();

    @Override
    public void robotInit() {
        // Put the mechanism to the dashboard so you can see it in Shuffleboard/Glass
        SmartDashboard.putData("Shooter Visual", m_mech);
    }

    @Override
    public void robotPeriodic() {
        double currentAngle = hood.getMeasurementDegrees();
        m_hoodVisual.setAngle(currentAngle);
    }
    

    public static void publish() {
            CommandScheduler.getInstance().run();
    }
}