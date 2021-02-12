package frc.team2485.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpiutil.math.MathUtil;
import frc.team2485.WarlordsLib.Tunable;
import frc.team2485.WarlordsLib.motorcontrol.PIDSparkMax;
import frc.team2485.WarlordsLib.robotConfigs.Configurable;
import frc.team2485.WarlordsLib.robotConfigs.LoadableConfigs;
import frc.team2485.WarlordsLib.robotConfigs.RobotConfigs;
import frc.team2485.WarlordsLib.robotConfigs.SavableConfigs;
import frc.team2485.robot.Constants;

public class Flywheels extends SubsystemBase implements Tunable, Configurable {

    private PIDSparkMax m_sparkLeft;
    private PIDSparkMax m_sparkRight;

    public Flywheels() {
        this.m_sparkLeft = new PIDSparkMax(Constants.Flywheels.SPARK_FLYWHEEL_LEFT_PORT, ControlType.kVelocity);
        this.m_sparkRight = new PIDSparkMax(Constants.Flywheels.SPARK_FLYWHEEL_RIGHT_PORT, ControlType.kVelocity);
        m_sparkRight.setInverted(true);

        m_sparkRight.getEncoder().setVelocityConversionFactor(Constants.Flywheels.GEAR_RATIO);
        m_sparkLeft.getEncoder().setVelocityConversionFactor(Constants.Flywheels.GEAR_RATIO);

        m_sparkLeft.setIdleMode(CANSparkMax.IdleMode.kCoast);
        m_sparkRight.setIdleMode(CANSparkMax.IdleMode.kCoast);

        m_sparkRight.setSmartCurrentLimit(Constants.Flywheels.SPARK_FLYWHEEL_LEFT_MAX_CURRENT);
        m_sparkLeft.setSmartCurrentLimit(Constants.Flywheels.SPARK_FLYWHEEL_RIGHT_MAX_CURRENT);

        m_sparkLeft.enableVoltageCompensation(Constants.NOMINAL_VOLTAGE);
        m_sparkRight.enableVoltageCompensation(Constants.NOMINAL_VOLTAGE);

        m_sparkRight.setTolerance(Constants.Flywheels.RPM_THRESHOLD);
        m_sparkLeft.setTolerance(Constants.Flywheels.RPM_THRESHOLD);

        RobotConfigs.getInstance().addConfigurable(Constants.Flywheels.LEFT_VELOCITY_CONTROLLER_CONFIGURABLE_LABEL, m_sparkLeft);
        RobotConfigs.getInstance().addConfigurable(Constants.Flywheels.RIGHT_VELOCITY_CONTROLLER_CONFIGURABLE_LABEL, m_sparkRight);

        this.addToShuffleboard();
    }

    private void addToShuffleboard() {
        ShuffleboardTab tab = Shuffleboard.getTab(Constants.Flywheels.TAB_NAME);
        tab.add("Left Flywheel Vel Ctrl", m_sparkLeft);
        tab.add("Right Flywheel Vel Ctrl", m_sparkRight);
        tab.addNumber("Left Flywheel Velocity", this::getLeftEncoderVelocity);
        tab.addNumber("Right Flywheel Velocity", this::getRightEncoderVelocity);
        tab.addNumber("Left Flywheel Current", m_sparkLeft::getOutputCurrent);
        tab.addNumber("Right Flywheel Current", m_sparkRight::getOutputCurrent);
        //tab.addNumber("Right Flywheel Error", this.getRightEncoderVelocity() - Constants.Setpoints.FARRPM);
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

    @Override
    public void resetPIDs() {
        m_sparkLeft.resetPID();
        m_sparkRight.resetPID();
    }

    private void setLeftVelocity(double velocity) {
        m_sparkLeft.runPID(MathUtil.clamp(velocity, Constants.Flywheels.FLYWHEELS_MIN_VELOCITY, Constants.Flywheels.FLYWHEELS_MAX_VELOCITY));
    }

    private void setRightVelocity(double velocity) {
        m_sparkRight.runPID(MathUtil.clamp(velocity, Constants.Flywheels.FLYWHEELS_MIN_VELOCITY, Constants.Flywheels.FLYWHEELS_MAX_VELOCITY));
    }

    public void setVelocity(double velocity) {

        setVelocity(velocity, velocity);
    }

    public void setVelocity(double leftVelocity, double rightVelocity) {
        setLeftVelocity(leftVelocity);
        setRightVelocity(rightVelocity);
    }

    public boolean atVelocitySetpoint() {
        return m_sparkLeft.atTarget() && m_sparkRight.atTarget();
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
    public void tunePeriodic(int layer) {
        m_sparkLeft.runPID();
        m_sparkRight.runPID();
    }

    @Override
    public void addChild(Object child) {

    }

    @Override
    public void loadConfigs(LoadableConfigs configs) {

    }

    @Override
    public void saveConfigs(SavableConfigs configs) {

    }
}
