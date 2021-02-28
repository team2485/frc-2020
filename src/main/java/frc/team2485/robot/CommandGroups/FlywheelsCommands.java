// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team2485.robot.CommandGroups;

import edu.wpi.first.wpilibj.command.CommandGroup;

public class FlywheelsCommands extends CommandGroup {
  /** Add your docs here. */
  public FlywheelsCommands(XboxController m_suraj, Flywheels m_flywheels) {
    new RunCommand(() -> {
      double output = 0;
      double pwm = Deadband.linearScaledDeadband(m_suraj.getTriggerAxis(GenericHID.Hand.kRight), Constants.OI.XBOX_DEADBAND);

      if (pwm != 0) {
          output = -4000 - pwm * 1000;

          m_flywheels.setVelocity(output);
      } else {
          m_flywheels.setPWM(0);
      }

  }, m_flywheels);
  
  }
}
