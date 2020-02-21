package frc.team2485.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.ControlType;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
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
        this.m_spark = new PIDSparkMax(Constants.Shooter.SPARK_FEEDER_PORT, ControlType.kPosition);
        this.m_spark.setSmartCurrentLimit(Constants.Shooter.SPARK_FEEDER_MAX_CURRENT, 20);

        SendableRegistry.add(m_spark, "Feeder Spark");
        RobotConfigs.getInstance().addConfigurable("feederSpark", m_spark);

        ShuffleboardTab tab = Shuffleboard.getTab("Shooter");
        tab.addNumber("Feeder Current", m_spark::getOutputCurrent);
    }

    public void setPWM(double pwm) {
        m_spark.set(pwm);
    }

    public double getEncoderPosition() {
        return m_spark.getEncoder().getPosition();
    }

    public double getEncoderVelocity() {
        return m_spark.getEncoder().getVelocity();
    }

    public CANEncoder getHoodEncoder() {
        return m_spark.getAlternateEncoder(Constants.Shooter.HOOD_ENCODER_CPR);
    }

}
