package frc.team2485.robot.subsystems.shooter;

import com.revrobotics.CANEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SendableRegistry;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team2485.WarlordsLib.motorcontrol.currentmanagement.CurrentSparkMax;
import frc.team2485.WarlordsLib.robotConfigs.RobotConfigs;
import frc.team2485.robot.Constants;

public class Flywheels extends SubsystemBase {
    private CurrentSparkMax m_sparkLeft;
    private CurrentSparkMax m_sparkRight;

    public Flywheels() {
        this.m_sparkLeft = new CurrentSparkMax(Constants.Shooter.SPARK_FLYWHEEL_LEFT_PORT, Constants.Shooter.SPARK_FLYWHEEL_LEFT_MAX_CURRENT);
        this.m_sparkRight = new CurrentSparkMax(Constants.Shooter.SPARK_FLYWHEEL_RIGHT_PORT, Constants.Shooter.SPARK_FLYWHEEL_RIGHT_MAX_CURRENT);
        m_sparkRight.setInverted(true);

        SendableRegistry.add(m_sparkLeft, "Flywheels Left Spark Current Controller");
        SendableRegistry.add(m_sparkRight, "Flywheels Right Spark Current Controller");
        RobotConfigs.getInstance().addConfigurable("flywheelsLeftSparkCurrentController", m_sparkLeft);
        RobotConfigs.getInstance().addConfigurable("flywheelsRightSparkCurrentController", m_sparkRight);
    }

    private void setLeftPWM(double pwm) {
        m_sparkLeft.setPWM(pwm);
    }

    private void setLeftCurrent(double current) {
        m_sparkLeft.setCurrent(current);
    }

    private void setRightPWM(double pwm) {
        m_sparkRight.setPWM(pwm);
    }

    private void setRightCurrent(double current) {
        m_sparkRight.setCurrent(current);
    }

    /**
     *
     * @param pwm
     * @param spinFactor factor that the right side is multiplied by to produce spin
     *                   > 1 meaning spins left
     *                   < 1 spins right
     */
    public void setPWM(double pwm, double spinFactor) {
        setLeftPWM(pwm);
        setRightPWM(pwm * spinFactor);
    }

    public void setPWM(double pwm) {
        setPWM(pwm, 1);
    }

    public void setCurrent(double current, double spinFactor) {
        setLeftCurrent(current);
        setRightCurrent(current * spinFactor);
    }

    public void setCurrent(double current) {
        setCurrent(current, 1);
    }

    public double getLeftEncoderVelocity() {
        return m_sparkLeft.getEncoder().getVelocity();
    }

    public double getRightEncoderVelocity() {
        return m_sparkRight.getEncoder().getVelocity();
    }

    public CANEncoder getHoodEncoder() {
        return m_sparkLeft.getAlternateEncoder();
    }
}
