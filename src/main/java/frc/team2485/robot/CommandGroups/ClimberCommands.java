// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team2485.robot.CommandGroups;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.team2485.robot.subsystems.Climber;

public class ClimberCommands extends CommandGroup {
  /** Add your docs here. */
  public ClimberCommands(Climber m_climber, Turret m_turret) {
   
    new ParallelCommandGroup(
      new ConditionalCommand(
              new TurretSetAngle(m_turret, 90, false),
              new TurretSetAngle(m_turret, -90, false),
              () -> m_turret.getEncoderPosition() > 0
      ),
      new SequentialCommandGroup(
              new WaitUntilCommand(
                      () -> Math.abs(m_turret.getEncoderPosition()) >= 45
              ),
              new InstantCommand(() -> m_climber.setPWM(Constants.Climber.DEFAULT_PWM)
              )
      )
  ); 

  }
}
