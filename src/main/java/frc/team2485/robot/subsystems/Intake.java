package frc.team2485.robot.subsystems;

import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.revrobotics.CANEncoder;
import edu.wpi.first.wpilibj.Counter;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalSource;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpiutil.math.MathUtil;
import frc.team2485.WarlordsLib.PositionPIDSubsystem;
import frc.team2485.WarlordsLib.Tunable;
import frc.team2485.WarlordsLib.VelocityPIDSubsystem;
import frc.team2485.WarlordsLib.control.WL_PIDController;
import frc.team2485.WarlordsLib.motorcontrol.PIDTalonSRX;
import frc.team2485.WarlordsLib.motorcontrol.WL_TalonSRX;
import frc.team2485.robot.Constants;

public class Intake extends SubsystemBase implements PositionPIDSubsystem, VelocityPIDSubsystem {
  private PIDTalonSRX m_talonX; //lateral movement belt
  private PIDTalonSRX m_talonZ; //center belt/z axis movement 

  private int ballsIntaken;

  public Intake() {
    this.m_talonX = new PID_TalonSRX(Constants.Intake.TALONX_PORT, ControlMode.Velocity);
    this.m_talonZ = new PID_TalonSRX(Constants.Intake.TALONZ_PORT, ControlMode.Velocity);

    this.m_TalonX.setTolerance(Constants.Intake.TOLERANCE);
    this.m_TalonY.setTolerance(Constants.Intake.TOLERANCE);
    this.m_talon.enableVoltageCompensation(Constants.NOMINAL_VOLTAGE);

  }

  private void addToShuffleboard() {
    ShuffleboardTab tab = Shuffleboard.getTab(Constants.Intake.TAB_NAME);
    tab.add("X Intake Rollers", m_talonX);
    tab.add("Y Intake Rollers", m_talonY);
    tab.add("X Intake Roller Velocity", this::getEncoderVelocityX);
    tab.add("Z Intake Roller Velocity", this::getEncoderVelocityZ);
  }

  public void setPWM(double pwmX, double pwmY) {
    m_talonX.set(pwmX);
    m_talonZ.set(pwmY);
  }

  public void resetPID() {
    m_talonX.resetPID();
    m_talonZ.resetPID();
  }

  public void runVelocityPID(double velocityX, double velocityY) {
    m_talonX.runPID(MathUtil.clamp(velocity, Constants.Intake.MIN_VELOCITY, Constants.Intake.MAX_VELOCITY));
    m_talonZ.runPID(MathUtil.clamp(velocityY, Constants.Intake.MIN_VELOCITY, Constants.Intake.MAX_VELOCITY));
  }

  public boolean atVelocitySetpoint() {
    return m_TalonX.atTarget() && m_TalonY.atTarget();
  }

  public double getEncoderVelocityX() {
    return m_talonX.getEncoderVelocity();
  }

  public double getEncoderVelocityY() {
    return m_talonY.getEncoderVelocity();
  }
}
