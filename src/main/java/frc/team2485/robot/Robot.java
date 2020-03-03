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
<<<<<<< HEAD

=======
>>>>>>> 3026135d9d4215d24e358273b03b94b6d31aa40b
    time = 0;
  }

  @Override
  public void autonomousPeriodic() {
<<<<<<< HEAD

=======
>>>>>>> 3026135d9d4215d24e358273b03b94b6d31aa40b
    //all of this is just for testing
    if(time<0.5) m_robotContainer.sendWidget1data(0,0);
    else if(time<1) m_robotContainer.sendWidget1data(0,120);
    else if(time<1.5)  m_robotContainer.sendWidget1data(0,240);
    else if(time<2)  m_robotContainer.sendWidget1data(0,0);
    else if(time<3) m_robotContainer.sendWidget1data(0,(time-3)*360);
    else if(time<3.5)  m_robotContainer.sendWidget1data(0,0);
    else if(time<4)  m_robotContainer.sendWidget1data(120,0);
    else if(time<4.5) m_robotContainer.sendWidget1data(240,0);
    else if(time<5) m_robotContainer.sendWidget1data(0,0);
    else if(time<6) m_robotContainer.sendWidget1data((time-6)*360,0);
<<<<<<< HEAD


=======
    
    
>>>>>>> 3026135d9d4215d24e358273b03b94b6d31aa40b
    if(time<0.5) m_robotContainer.sendWidget2data(Math.PI/4.0,5);
    else if(time<1) m_robotContainer.sendWidget2data(Math.PI/4.0,10);
    else if(time<1.5)  m_robotContainer.sendWidget2data(Math.PI/4.0,20);
    else if(time<2)  m_robotContainer.sendWidget2data(Math.PI/4.0,30);
    else if(time<3) m_robotContainer.sendWidget2data(Math.PI/4.0,(time-3)*30);
    else if(time<3.5)  m_robotContainer.sendWidget2data(0,15);
    else if(time<4)  m_robotContainer.sendWidget2data(Math.PI/6.0,15);
    else if(time<4.5) m_robotContainer.sendWidget2data(Math.PI/3.0,15);
    else if(time<5) m_robotContainer.sendWidget2data(Math.PI/2.0,15);
    else if(time<6) m_robotContainer.sendWidget2data((time-6)*Math.PI/2.0,15);
    //end of testing code, comment out later
    time+=0.02;
//     m_robotContainer.widget1NetworkTables();
//     m_robotContainer.widget2NetworkTables();
<<<<<<< HEAD

=======
>>>>>>> 3026135d9d4215d24e358273b03b94b6d31aa40b
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
