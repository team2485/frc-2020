// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team2485.robot.CommandGroups;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.team2485.robot.subsystems.Drivetrain;
import frc.team2485.robot.subsystems.Flywheels;
import frc.team2485.robot.subsystems.HighMagazine;
import frc.team2485.robot.subsystems.LowMagazine;

public class AutoCommand extends CommandGroup {
  /** Add your docs here. */
  public AutoCommand(Flywheels m_flywheels, Turret m_turret, Feeder m_feeder, HighMagazine m_highMagazine, LowMagazine m_lowMagazine, Drivetrain m_drivetrain, Hood m_hood) {
   
    new SequentialCommandGroup(
//                                new WaitUntilCommand(() -> m_flywheels.getLeftEncoderVelocity() < -2000),
//                new ParallelDeadlineGroup(
//                        new WaitCommand(2),
//                        new SetFlywheels(m_flywheels, () -> Constants.Setpoints.INITIATION_LINE.RPM) //edit
////                        new TurretSetAngle(m_turret, () -> {
////                            return m_turret.getEncoderPosition()
////                                    + m_turret.getLimelight().getTargetHorizontalOffset(0);
////                        })
//                ),
                new WaitCommand(3),
                new InstantCommand(() -> {
//                    m_lowMagazine.setPWM(-0.5);
                    m_feeder.setPWM(-0.9);
                }),
                //new WaitCommand(Constants.Magazine.NORMAL_BALL_INCREMENT_TIMEOUT),
                new IncrementHighMagazine(m_highMagazine, Constants.Magazine.HIGH_INDEX_BY_ONE_POS),
                new WaitCommand(Constants.Magazine.NORMAL_BALL_INCREMENT_TIMEOUT),
                new IncrementHighMagazine(m_highMagazine, Constants.Magazine.HIGH_INDEX_BY_ONE_POS),
                new WaitCommand(Constants.Magazine.NORMAL_BALL_INCREMENT_TIMEOUT),
                new IncrementHighMagazine(m_highMagazine, Constants.Magazine.HIGH_INDEX_BY_ONE_POS),
                new WaitCommand(Constants.Magazine.NORMAL_BALL_INCREMENT_TIMEOUT),
                new IncrementHighMagazine(m_highMagazine, Constants.Magazine.HIGH_INDEX_BY_ONE_POS),
                new InstantCommand(() -> {
                    m_lowMagazine.setPWM(0);
                    m_highMagazine.setPWM(0);
                    m_feeder.setPWM(0);
                    m_flywheels.setPWM(0);
                }),
                new RunCommand(
                        () -> {
                            m_drivetrain.curvatureDrive(-0.3, 0, false);
                        }
                )
                        .withTimeout(2)
                        .andThen(
                                new InstantCommand(() -> {
                                    m_drivetrain.curvatureDrive(0, 0, false);
                                })
                        ))
                .alongWith(
                        new SetHood(m_hood, () -> Constants.Setpoints.INITIATION_LINE.ANGLE, true)
                )
                .alongWith(
                        new SetFlywheels(m_flywheels, () -> Constants.Setpoints.INITIATION_LINE.RPM)
                )
                .alongWith(
                        new RunCommand(() -> {
                            m_turret.setPWM(0);
                        }, m_turret)
                );

  }
}
