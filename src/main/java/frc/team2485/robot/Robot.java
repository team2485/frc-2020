/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team2485.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.team2485.WarlordsLib.robotConfigs.RobotConfigs;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  @Override
  public void robotInit() {
    RobotConfigs.getInstance().loadConfigsFromFile(Constants.CONFIGS_FILE);
    m_robotContainer = new RobotContainer();

    CameraServer.getInstance().startAutomaticCapture();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {
    // COMMENT OUT TO FREEZE!
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
    m_robotContainer.autoInit();
    m_robotContainer.gameInit();
  }

  @Override
  public void autonomousPeriodic() {


  }

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    m_robotContainer.teleopInit();
    m_robotContainer.gameInit();
  }

  @Override
  public void teleopPeriodic() {

  }

  @Override
  public void testInit() {

    RobotConfigs.getInstance().loadConfigsFromFile(Constants.CONFIGS_FILE);
    CommandScheduler.getInstance().cancelAll();
    m_robotContainer.testInit();
    m_robotContainer.gameInit();

  }

  @Override
  public void testPeriodic() {
    m_robotContainer.testPeriodic();



  }
}
