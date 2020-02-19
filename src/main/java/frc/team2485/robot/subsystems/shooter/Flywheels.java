package frc.team2485.robot.subsystems.shooter;

import com.revrobotics.CANEncoder;
import com.revrobotics.ControlType;
import edu.wpi.first.wpilibj.smartdashboard.SendableRegistry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team2485.WarlordsLib.motorcontrol.PIDSparkMax;
import frc.team2485.WarlordsLib.motorcontrol.currentmanagement.CurrentSparkMax;
import frc.team2485.WarlordsLib.robotConfigs.RobotConfigs;
import frc.team2485.robot.Constants;

public class Flywheels extends SubsystemBase {
    private PIDSparkMax m_sparkLeft;
    private PIDSparkMax m_sparkRight;

    public Flywheels() {
        this.m_sparkLeft = new PIDSparkMax(Constants.Shooter.SPARK_FLYWHEEL_LEFT_PORT, ControlType.kCurrent);
        this.m_sparkRight = new PIDSparkMax(Constants.Shooter.SPARK_FLYWHEEL_RIGHT_PORT, ControlType.kCurrent);
        m_sparkRight.setInverted(true);

        m_sparkRight.getEncoder().setVelocityConversionFactor(2);
        m_sparkLeft.getEncoder().setVelocityConversionFactor(2);

        SendableRegistry.add(m_sparkLeft, "Flywheels Left Spark Current Controller");
        SendableRegistry.add(m_sparkRight, "Flywheels Right Spark Current Controller");
        RobotConfigs.getInstance().addConfigurable("flywheelsLeftSparkCurrentController", m_sparkLeft);
        RobotConfigs.getInstance().addConfigurable("flywheelsRightSparkCurrentController", m_sparkRight);
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
}
