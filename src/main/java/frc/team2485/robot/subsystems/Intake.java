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
import frc.team2485.robot.Constants;

public class Intake extends SubsystemBase implements Tunable{
  private PIDTalonSRX m_talonX; //lateral movement belt
  private PIDTalonSRX m_talonZ; //center belt/z axis movement 

  public Intake() {
    this.m_talonX = new PIDTalonSRX(Constants.Intake.TALONX_PORT, ControlMode.Velocity);
    this.m_talonZ = new PIDTalonSRX(Constants.Intake.TALONZ_PORT, ControlMode.Velocity);

    this.m_talonX.setTolerance(Constants.Intake.TOLERANCE);
    this.m_talonZ.setTolerance(Constants.Intake.TOLERANCE);
    this.m_talonX.enableVoltageCompensation(Constants.NOMINAL_VOLTAGE);
    this.m_talonZ.enableVoltageCompensation(Constants.NOMINAL_VOLTAGE);

    RobotConfigs.getInstance().addConfigurable(Constants.Intake.X_VELOCITY_CONTROLLER_CONFIGURABLE_LABEL, m_talonX);
    RobotConfigs.getInstance().addConfigurable(Constants.Intake.Z_VELOCITY_CONTROLLER_CONFIGURABLE_LABEL, m_talonZ);
  }

  private void addToShuffleboard() {
    ShuffleboardTab tab = Shuffleboard.getTab(Constants.Intake.TAB_NAME);
    tab.add("X Intake Rollers", m_talonX);
    tab.add("Z Intake Rollers", m_talonZ);
    tab.addNumber("X Intake Roller Velocity", this::getEncoderVelocityX);
    tab.addNumber("Z Intake Roller Velocity", this::getEncoderVelocityZ);
  }

  public void setPWM(double pwm) {
    m_talonX.set(pwm);
    m_talonZ.set(pwm);
  }

  public void setPWM(double pwmX, double pwmY) {
    m_talonX.set(pwmX);
    m_talonZ.set(pwmY);
  }

  public void resetPIDs() {
    m_talonX.resetPID();
    m_talonZ.resetPID();
  }

  public void runVelocityPID(double velocityX, double velocityY) {
    m_talonX.runPID(MathUtil.clamp(velocityX, Constants.Intake.MIN_VELOCITY, Constants.Intake.MAX_VELOCITY));
    m_talonZ.runPID(MathUtil.clamp(velocityY, Constants.Intake.MIN_VELOCITY, Constants.Intake.MAX_VELOCITY));
  }

  public boolean atVelocitySetpoint() {
    return m_talonX.atTarget() && m_talonZ.atTarget();
  }

  public double getEncoderVelocityX() {
    return m_talonX.getEncoderVelocity();
  }

  public double getEncoderVelocityZ() {
    return m_talonZ.getEncoderVelocity();
  }

  /**
     * Should run periodically and run the motor to tune when enabled
     */
    @Override
    public void tunePeriodic(int layer) {
        m_talonX.runPID();
        m_talonZ.runPID();

    }
}
