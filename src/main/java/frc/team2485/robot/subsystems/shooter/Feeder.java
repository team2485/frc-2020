package frc.team2485.robot.subsystems.shooter;

import edu.wpi.first.wpilibj.smartdashboard.SendableRegistry;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team2485.WarlordsLib.motorcontrol.currentmanagement.CurrentSparkMax;
import frc.team2485.WarlordsLib.robotConfigs.RobotConfigs;
import frc.team2485.robot.Constants;

public class Feeder extends SubsystemBase {
    private CurrentSparkMax m_spark;

    public Feeder() {
        this.m_spark = new CurrentSparkMax(Constants.Shooter.SPARK_FEEDER_PORT, Constants.Shooter.SPARK_FEEDER_MAX_CURRENT);
        SendableRegistry.add(m_spark, "Feeder Spark Current Controller");
        RobotConfigs.getInstance().addConfigurable("feederSparkCurrentController", m_spark);
    }

    public void setPwM(double pwm) {
        m_spark.setPWM(pwm);
    }

    public void setCurrent(double current) {
        m_spark.setCurrent(current);
    }

    public double getEncoderPosition() {
        return m_spark.getEncoder().getPosition();
    }

    public double getEncoderVelocity() {
        return m_spark.getEncoder().getVelocity();
    }
}
