/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team2485.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.team2485.WarlordsLib.oi.WL_XboxController;
import frc.team2485.robot.Subsystems.Drivetrain;

public class RobotContainer {

  private WL_XboxController xbox;
  private Drivetrain drivetrain;

  public RobotContainer() {
    configureButtonBindings();
  }


  private void configureButtonBindings() {

    drivetrain = new Drivetrain();

  }

  public void configureCommands(){

    drivetrain.setDefaultCommand(new RunCommand(() ->
    drivetrain.curvatureDrive(xbox.getTriggerAxis(GenericHID.Hand.kRight)-xbox.getTriggerAxis(GenericHID.Hand.kLeft), xbox.getX(GenericHID.Hand.kLeft), xbox.getXButton())));

  }


//  public Command getAutonomousCommand() {
//    return m_autoCommand;
//  }
}
