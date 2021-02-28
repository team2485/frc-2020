// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team2485.robot.CommandGroups;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.team2485.robot.subsystems.BallCounter;
import frc.team2485.robot.subsystems.Flywheels;
import frc.team2485.robot.subsystems.HighMagazine;
import frc.team2485.robot.subsystems.LowMagazine;

public class FeedBallToShooter extends CommandGroup {
  /** Add your docs here. */
  public FeedBallToShooter(HighMagazine m_HighMagazine, LowMagazine m_LowMagazine, Flywheels m_Flywheels, Feeder m_feeder, BallCounter m_BallCounter) {

    new ConditionalCommand(
      new SequentialCommandGroup(
              new WaitUntilCommand(() -> m_flywheels.atVelocitySetpoint()),
              new InstantCommand(
                      () -> {
                          m_lowMagazine.setPWM(-0.5);
                          m_feeder.setPWM(-0.9); //change
                      }, m_lowMagazine, m_feeder
              ),
//                                new WaitCommand(Constants.Magazine.NORMAL_BALL_INCREMENT_TIMEOUT),
              new IncrementHighMagazine(m_highMagazine, Constants.Magazine.HIGH_INDEX_BY_ONE_POS)
      ),
      new InstantCommand(),
      () -> m_flywheels.atVelocitySetpoint());

//                new SequentialCommandGroup(
//                        new RunCommand(
//                                () -> {
//                                    m_highMagazine.runVelocityPID(-40);
//                                }
//                        ).withInterrupt(
//                                () -> m_ballCounter.exitIRHasBall()
//                        ).andThen(
//                                new InstantCommand(
//                                        () -> {
//                                            m_highMagazine.setPWM(0);
//                                        }
//                                )
//                        ),
//                        new ConditionalCommand(
//                                new SequentialCommandGroup(
//                                        new InstantCommand(() -> {
//                                            m_lowMagazine.setPWM(-0.5);
//                                            m_feeder.setPWM(-0.8);
//                                        }).alongWith(
//                                                new IncrementHighMagazine(m_highMagazine, Constants.Magazine.HIGH_INDEX_BY_ONE_POS),
//                                                new WaitCommand(Constants.Magazine.NORMAL_BALL_INCREMENT_TIMEOUT),
//                                        )),
//                                new InstantCommand(),
//                                () -> m_flywheels.getLeftEncoderVelocity() < -1000 && m_flywheels.getRightEncoderVelocity() < -1000
//                        )
//                ),
  }
}
