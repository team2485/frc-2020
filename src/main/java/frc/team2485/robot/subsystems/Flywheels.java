package frc.team2485.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.ControlType;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableRegistry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team2485.WarlordsLib.Tunable;
import frc.team2485.WarlordsLib.motorcontrol.PIDSparkMax;
import frc.team2485.WarlordsLib.motorcontrol.currentmanagement.CurrentSparkMax;
import frc.team2485.WarlordsLib.robotConfigs.RobotConfigs;
import frc.team2485.robot.Constants;

public class Flywheels extends SubsystemBase implements Tunable {

    private PIDSparkMax m_sparkLeft;
    private PIDSparkMax m_sparkRight;

    public Flywheels() {
        this.m_sparkLeft = new PIDSparkMax(Constants.Shooter.SPARK_FLYWHEEL_LEFT_PORT, ControlType.kVelocity);
        this.m_sparkRight = new PIDSparkMax(Constants.Shooter.SPARK_FLYWHEEL_RIGHT_PORT, ControlType.kVelocity);
        m_sparkRight.setInverted(true);

        m_sparkRight.getEncoder().setVelocityConversionFactor(2);
        m_sparkLeft.getEncoder().setVelocityConversionFactor(2);

        m_sparkLeft.enableVoltageCompensation(Constants.NOMINAL_VOLTAGE);
        m_sparkRight.enableVoltageCompensation(Constants.NOMINAL_VOLTAGE);

        RobotConfigs.getInstance().addConfigurable(Constants.Shooter.LEFT_FLYWHEEL_VELOCITY_CONTROLLER_CONFIGURABLE_LABEL, m_sparkLeft);
        RobotConfigs.getInstance().addConfigurable(Constants.Shooter.RIGHT_FLYWHEEL_VELOCITY_CONTROLLER_CONFIGURABLE_LABEL, m_sparkRight);

        this.addToShuffleboard();
    }

    private void addToShuffleboard() {
        ShuffleboardTab tab = Shuffleboard.getTab(Constants.Shooter.TAB_NAME);
        tab.addNumber("Flywheels Velocity Setpoint", m_sparkLeft::getSetpoint);
        tab.addNumber("Left Flywheel Velocity", this::getLeftEncoderVelocity);
        tab.addNumber("Right Flywheel Velocity", this::getRightEncoderVelocity);
        tab.addNumber("Left Flywheel Current", m_sparkLeft::getOutputCurrent);
        tab.addNumber("Right Flywheel Current", m_sparkRight::getOutputCurrent);
        tab.add("Flywheels Left Spark", m_sparkLeft);
        tab.add("Flywheels Right Spark", m_sparkRight);
    }

    private void setLeftPWM(double pwm) {
        m_sparkLeft.set(pwm);
    }

    private void setRightPWM(double pwm) {
        m_sparkRight.set(pwm);
    }

    public void setPWM(double leftPWM, double rightPWM) {
        setLeftPWM(leftPWM);
        setRightPWM(rightPWM);
    }

    public void setPWM(double pwm) {
        setPWM(pwm, pwm);
    }

    private void setLeftVelocity(double velocity) {
        m_sparkLeft.runPID(velocity);
    }

    private void setRightVelocity(double velocity) {
        m_sparkRight.runPID(velocity);
    }

    public void setVelocity(double velocity) {
        setVelocity(velocity, velocity);
    }

    public void setVelocity(double leftVelocity, double rightVelocity) {
        setLeftVelocity(leftVelocity);
        setRightVelocity(rightVelocity);
    }

    public double getLeftEncoderVelocity() {
        return m_sparkLeft.getEncoder().getVelocity();
    }

    public double getRightEncoderVelocity() {
        return m_sparkRight.getEncoder().getVelocity();
    }


    @Override
    public void periodic() {
    }

    @Override
    public void tunePeriodic() {
        m_sparkLeft.runPID();
        m_sparkRight.runPID();
    }
}
