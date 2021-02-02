// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team2485.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team2485.WarlordsLib.motorcontrol.PIDSparkMax;

public class Intake extends SubsystemBase implements VelocityPIDSubsystem {
  private PIDSparkMax leftSpark;
  private PIDSparkMax rightSpark;

  /** Creates a new Intake. */
  public Intake() {
    leftSpark = new PIDSparkMax(Constants.Intake.SPARK_LEFT_PORT);
    rightSpark = new PIDSparkMax(Constants.Intake.SPARK_RIGHT_PORT);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
