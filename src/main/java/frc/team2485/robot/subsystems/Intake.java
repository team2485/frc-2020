package frc.team2485.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
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
import frc.team2485.WarlordsLib.robotConfigs.RobotConfigs;
import frc.team2485.WarlordsLib.robotConfigs.Configurable;
import frc.team2485.WarlordsLib.robotConfigs.LoadableConfigs;
import frc.team2485.WarlordsLib.robotConfigs.RobotConfigs;
import frc.team2485.WarlordsLib.robotConfigs.SavableConfigs;
import frc.team2485.WarlordsLib.Debounce;
import frc.team2485.WarlordsLib.ArmAndTrigger;
import frc.team2485.robot.Constants;
import frc.team2485.WarlordsLib.motorcontrol.PIDSparkMax;


import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpiutil.math.MathUtil;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.team2485.WarlordsLib.Tunable;
import frc.team2485.WarlordsLib.motorcontrol.PIDSparkMax;
import frc.team2485.WarlordsLib.robotConfigs.Configurable;
import frc.team2485.WarlordsLib.robotConfigs.LoadableConfigs;
import frc.team2485.WarlordsLib.robotConfigs.RobotConfigs;
import frc.team2485.WarlordsLib.robotConfigs.SavableConfigs;
import frc.team2485.WarlordsLib.Debounce;
import frc.team2485.WarlordsLib.ArmAndTrigger;
import frc.team2485.robot.Constants;


public class Intake extends SubsystemBase implements Tunable{
  private PIDSparkMax m_sparkX; //lateral movement belt
  private PIDSparkMax m_sparkZ; //center belt/z axis movement

  public Intake() {
    this.m_sparkX = new PIDSparkMax(Constants.Intake.SPARKX_PORT, ControlType.kVelocity);
    this.m_sparkZ = new PIDSparkMax(Constants.Intake.SPARKZ_PORT, ControlType.kVelocity);

    this.m_sparkX.setTolerance(Constants.Intake.TOLERANCE);
    this.m_sparkZ.setTolerance(Constants.Intake.TOLERANCE);
    this.m_sparkX.enableVoltageCompensation(Constants.NOMINAL_VOLTAGE);
    this.m_sparkZ.enableVoltageCompensation(Constants.NOMINAL_VOLTAGE);

    RobotConfigs.getInstance().addConfigurable(Constants.Intake.X_VELOCITY_CONTROLLER_CONFIGURABLE_LABEL, m_sparkX);
    RobotConfigs.getInstance().addConfigurable(Constants.Intake.Z_VELOCITY_CONTROLLER_CONFIGURABLE_LABEL, m_sparkZ);

    this.addToShuffleboard();

  }

  private void addToShuffleboard() {
    ShuffleboardTab tab = Shuffleboard.getTab(Constants.Intake.TAB_NAME);
    tab.add("X Intake Rollers", m_sparkX);
    tab.add("Z Intake Rollers", m_sparkZ);
    tab.addNumber("X Intake Roller Velocity", this::getEncoderVelocityX);
    tab.addNumber("Z Intake Roller Velocity", this::getEncoderVelocityZ);
  }

  public void setPWM(double pwm) {
    m_sparkX.set(pwm);
    m_sparkZ.set(pwm);
  }

  public void setPWM(double pwmX, double pwmY) {
    m_sparkX.set(pwmX);
    m_sparkZ.set(pwmY);
  }

  public void resetPIDs() {
    m_sparkX.resetPID();
    m_sparkZ.resetPID();
  }

  public void runVelocityPID(double velocityX, double velocityZ) {
    m_sparkX.runPID(MathUtil.clamp(velocityX, Constants.Intake.MIN_VELOCITY, Constants.Intake.MAX_VELOCITY));
    m_sparkZ.runPID(MathUtil.clamp(velocityZ, Constants.Intake.MIN_VELOCITY, Constants.Intake.MAX_VELOCITY));
  }

  public boolean atVelocitySetpoint() {
    return m_sparkX.atTarget() && m_sparkZ.atTarget();
  }

  public double getEncoderVelocityX() {
    return m_sparkX.getSensorOutput();
  }

  public double getEncoderVelocityZ() {
    return m_sparkZ.getSensorOutput();
  }

  /**
     * Should run periodically and run the motor to tune when enabled
     */
    @Override
    public void tunePeriodic(int layer) {
        m_sparkX.runPID();
        m_sparkZ.runPID();
    }
}
