// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team2485.robot.CommandGroups;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.team2485.robot.subsystems.LowMagazine;

public class RunLowMag extends CommandGroup {

  public RunLowMag(LowMagazine m_LowMagazine) {
   
//                new ConditionalCommand(
//                        new InstantCommand(() -> {
////                            m_lowMagazine.setPWM(Constants.Magazine.LOW_BELT_INTAKE_PWM);
//                            m_lowMagazine.runVelocityPID(-70);
//
//                        }),
//                        new InstantCommand(() -> {
//                            m_lowMagazine.setPWM(0);
//                        }),
//                        () -> {
//                            return !((m_ballCounter.getNumBallsHigh() >= Constants.Magazine.HIGH_MAGAZINE_BALL_CAPACITY
//                                    || (m_ballCounter.getNumBallsHigh() >= 3 && m_ballCounter.exitIRHasBall()))
//                                    && m_ballCounter.transferIRHasBall());
////                            return true;
//                        }
//                )
new RunCommand(() -> {
  //m_lowMagazine.setPWM(Constants.Magazine.LOW_BELT_INTAKE_PWM);
  m_lowMagazine.runVelocityPID(Constants.Magazine.LOW_INTAKE_VELOCITY);
  
  });

  }
}
