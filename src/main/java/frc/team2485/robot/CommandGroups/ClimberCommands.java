// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team2485.robot.CommandGroups;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ClimberCommands extends ParallelCommandGroup {
  /** Creates a new ClimberCommands. */
  public ClimberCommands(Climber m_climber, Turret m_turret) {
   
    addCommands(

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
