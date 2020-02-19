package frc.team2485.robot.subsystems.shooter;

import com.revrobotics.ControlType;
import edu.wpi.first.wpilibj.smartdashboard.SendableRegistry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team2485.WarlordsLib.motorcontrol.PIDSparkMax;
import frc.team2485.WarlordsLib.motorcontrol.WL_SparkMax;
import frc.team2485.WarlordsLib.motorcontrol.currentmanagement.CurrentSparkMax;
import frc.team2485.WarlordsLib.robotConfigs.RobotConfigs;
import frc.team2485.robot.Constants;

public class Feeder extends SubsystemBase {
    private PIDSparkMax m_spark;

    public Feeder() {
        this.m_spark = new PIDSparkMax(Constants.Shooter.SPARK_FEEDER_PORT, ControlType.kCurrent);
        SendableRegistry.add(m_spark, "Feeder Spark");
        RobotConfigs.getInstance().addConfigurable("feederSparkCurrentController", m_spark);

    }

    public void setPwM(double pwm) {
        m_spark.set(pwm);
    }

//    public void setCurrent(double current) {
//        m_spark.setCurrent(current);
//    }

    public double getEncoderPosition() {
        return m_spark.getEncoder().getPosition();
    }

    public double getEncoderVelocity() {
        return m_spark.getEncoder().getVelocity();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("SPARK FEEDER CURRENT", m_spark.getOutputCurrent());
    }
}
