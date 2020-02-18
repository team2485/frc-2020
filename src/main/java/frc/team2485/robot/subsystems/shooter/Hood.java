package frc.team2485.robot.subsystems.shooter;

import com.revrobotics.CANEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SendableRegistry;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team2485.WarlordsLib.motorcontrol.currentmanagement.CurrentSparkMax;
import frc.team2485.WarlordsLib.robotConfigs.RobotConfigs;
import frc.team2485.robot.Constants;


public class Hood extends SubsystemBase {
    private CurrentSparkMax m_spark;
    private CANEncoder m_hoodEncoder;

    public Hood(CANEncoder hoodEncoder) {
        this.m_spark = new CurrentSparkMax(Constants.Shooter.SPARK_PIVOT_PORT, Constants.Shooter.SPARK_PIVOT_MAX_CURRENT);
        this.m_hoodEncoder = hoodEncoder;

        SendableRegistry.add(m_spark, "Hood Spark Current Controller");
        RobotConfigs.getInstance().addConfigurable("hoodSparkCurrentController", m_spark);


    }

    public void setPWM(double pwm) {
        m_spark.setPWM(pwm);
    }

    public void setCurrent(double current) {
        m_spark.setCurrent(current);
    }

    public double getEncoderPosition() {
        return m_hoodEncoder.getPosition();
    }
}
