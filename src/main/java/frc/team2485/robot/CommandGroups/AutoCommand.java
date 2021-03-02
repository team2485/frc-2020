// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team2485.robot.CommandGroups;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.team2485.robot.subsystems.Drivetrain;
import frc.team2485.robot.subsystems.Flywheels;
import frc.team2485.robot.subsystems.HighMagazine;
import frc.team2485.robot.subsystems.Hood;
import frc.team2485.robot.subsystems.LowMagazine;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoCommand extends ParallelCommandGroup {
  /** Creates a new AutoCommand. */
  public AutoCommand(Flywheels m_flywheels, Turret m_turret, Feeder m_feeder, HighMagazine m_highMagazine, LowMagazine m_lowMagazine, Drivetrain m_drivetrain, Hood m_hood) {
    
    addCommands(

        new SequentialCommandGroup(               
                                new WaitCommand(3),
                                new InstantCommand(() -> {
                                    m_feeder.setPWM(-0.9);
                                }),
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
                                        )),
        new SetHood(m_hood, () -> Constants.Setpoints.INITIATION_LINE.ANGLE, true),
        new SetFlywheels(m_flywheels, () -> Constants.Setpoints.INITIATION_LINE.RPM),
        new RunCommand(() -> {
                        m_turret.setPWM(0);
                }, m_turret));
  }
}
