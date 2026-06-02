package com.swrobotics.robot.logging;


import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class RobotView extends TimedRobot {
    // Visualization objects
    private final Mechanism2d m_mech = new Mechanism2d(60, 60);



    @Override
    public void robotInit() {
        // Put the mechanism to the dashboard so you can see it in Shuffleboard/Glass
        SmartDashboard.putData("Shooter Visual", m_mech);
    }

    @Override
    public void robotPeriodic() {

    }
    

    public static void publish() {
            CommandScheduler.getInstance().run();
    }
}