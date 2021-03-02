// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team2485.robot.CommandGroups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.team2485.robot.subsystems.Flywheels;
import frc.team2485.robot.subsystems.HighMagazine;
import frc.team2485.robot.subsystems.LowMagazine;
import jdk.internal.jline.internal.Log;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class FeedBallToShooterTest extends SequentialCommandGroup {
  /** Creates a new FeedBallToShooterTest. */
  public FeedBallToShooterTest(Flywheels m_flywheels, LowMagazine m_lowMagazine, Feeder m_feeder, HighMagazine m_highMagazine) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(

              new WaitUntilCommand(() -> m_flywheels.atVelocitySetpoint()),
              new InstantCommand(
                        () -> {
                            m_lowMagazine.setPWM(-0.5);
                            m_feeder.setPWM(-0.9); //change
                        }, m_lowMagazine, m_feeder
                ),
                new IncrementHighMagazine(m_highMagazine, Constants.Magazine.HIGH_INDEX_BY_ONE_POS)
        );
       
  }
}
