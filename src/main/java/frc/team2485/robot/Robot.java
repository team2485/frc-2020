/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team2485.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.team2485.WarlordsLib.robotConfigs.RobotConfigs;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;
  
  private double time;

  @Override
  public void robotInit() {
    RobotConfigs.getInstance().loadConfigsFromFile(Constants.CONFIGS_FILE);
    m_robotContainer = new RobotContainer();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {
    RobotConfigs.getInstance().saveConfigsToFile(Constants.CONFIGS_FILE);
  }

  @Override
  public void disabledPeriodic() {
  }

  @Override
  public void autonomousInit() {
    //CHANGE THIS LATER
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
    time = 0;
  }

  @Override
  public void autonomousPeriodic() {
    if(time<0.5) m_robotContainer.getTurret().resetEncoderPosition(0);
    else if(time<1) m_robotContainer.getTurret().resetEncoderPosition(2*Math.PI/3.0);
    else if(time<1.5) m_robotContainer.getTurret().resetEncoderPosition(4*Math.PI/3.0);
    else if(time<2) m_robotContainer.getTurret().resetEncoderPosition(0);
    else if(time<3) m_robotContainer.getTurret().resetEncoderPosition(m_robotContainer.getTurret().getEncoderPosition()+2*Math.PI/50.0);
    else if(time<3.5) m_robotContainer.getDrivetrain().setHeading(0);
    else if(time<4) m_robotContainer.getDrivetrain().setHeading(2*Math.PI/3.0);
    else if(time<4.5) m_robotContainer.getDrivetrain().setHeading(4*Math.PI/3.0);
    else if(time<5) m_robotContainer.getDrivetrain().setHeading(0);
    else if(time<6) m_robotContainer.getDrivetrain().setHeading(m_robotContainer.getDrivetrain().getHeading()+2*Math.PI/50.0);
    
    
//     if(time<0.5) m_robotContainer.getTurret().resetEncoderPosition(0);
//     else if(time<1) m_robotContainer.getTurret().resetEncoderPosition(2*Math.PI/3.0);
//     else if(time<1.5) m_robotContainer.getTurret().resetEncoderPosition(4*Math.PI/3.0);
//     else if(time<2) m_robotContainer.getTurret().resetEncoderPosition(0);
//     else if(time<3) m_robotContainer.getTurret().resetEncoderPosition(m_robotContainer.getTurret().getEncoderPosition()+2*Math.PI/50.0);
//     else if(time<3.5) m_robotContainer.getDrivetrain().setHeading(0);
//     else if(time<4) m_robotContainer.getDrivetrain().setHeading(2*Math.PI/3.0);
//     else if(time<4.5) m_robotContainer.getDrivetrain().setHeading(4*Math.PI/3.0);
//     else if(time<5) m_robotContainer.getDrivetrain().setHeading(0);
//     else if(time<6) m_robotContainer.getDrivetrain().setHeading(m_robotContainer.getDrivetrain().getHeading()+2*Math.PI/50.0);
    time+=0.02;
    m_robotContainer.widgetNetworkTables();
  }

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {

  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
    m_robotContainer.testInit();
  }

  @Override
  public void testPeriodic() {
    m_robotContainer.testPeriodic();
  }
}
