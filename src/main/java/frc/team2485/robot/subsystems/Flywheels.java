package frc.team2485.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.ControlType;
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

        SendableRegistry.add(m_sparkLeft, "Flywheels Left Spark Velocity Controller");
        SendableRegistry.add(m_sparkRight, "Flywheels Right Spark Velocity Controller");
        RobotConfigs.getInstance().addConfigurable("flywheelsLeftSparkVelocityController", m_sparkLeft);
        RobotConfigs.getInstance().addConfigurable("flywheelsRightSparkVelocityController", m_sparkRight);
    }

    private void setLeftPWM(double pwm) {
        m_sparkLeft.set(pwm);
    }

    private void setRightPWM(double pwm) {
        m_sparkRight.set(pwm);
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

    private void setLeftVelocity(double velocity) {
        m_sparkLeft.runPID(velocity);
    }

    private void setRightVelocity(double velocity) {
        m_sparkRight.runPID(velocity);
    }

    public void setVelocity(double velocity) {
        setVelocity(velocity, 1);
    }

    public void setVelocity(double velocity, double spinFactor) {
        setLeftVelocity(velocity);
        setRightVelocity(velocity * spinFactor);
    }

    public double getLeftEncoderVelocity() {
        return m_sparkLeft.getEncoder().getVelocity();
    }

    public double getRightEncoderVelocity() {
        return m_sparkRight.getEncoder().getVelocity();
    }
    

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Left RPM", m_sparkLeft.getEncoder().getVelocity());
        SmartDashboard.putNumber("Right RPM", m_sparkRight.getEncoder().getVelocity());
    }

    public void tunePeriodic() {
        m_sparkLeft.runPID();
        m_sparkRight.runPID();
    }
}
