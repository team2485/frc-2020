// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team2485.robot.CommandGroups;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.team2485.robot.subsystems.HighMagazine;

public class IntakingCommands extends CommandGroup {


  public IncrementHighMag(HighMagazine m_highMagazine) {
    new ConditionalCommand(
      new IncrementHighMagazine(m_highMagazine, Constants.Magazine.HIGH_INDEX_BY_ONE_POS),
      new InstantCommand(() -> {
          m_highMagazine.setPWM(0);
      }, m_highMagazine),
      () -> {
          return
                  m_ballCounter.transferIRHasBall();
       
      });
  }
}
