package frc.team2485.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.ControlType;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team2485.WarlordsLib.motorcontrol.PIDSparkMax;
import frc.team2485.WarlordsLib.robotConfigs.RobotConfigs;
import frc.team2485.robot.Constants;
import frc.team2485.robot.commands.shooter.Shoot;

public class Feeder extends SubsystemBase {
    private PIDSparkMax m_spark;

    public Feeder() {
        this.m_spark = new PIDSparkMax(Constants.Shooter.SPARK_FEEDER_PORT, ControlType.kVelocity);
        this.m_spark.setSmartCurrentLimit(Constants.Shooter.SPARK_FEEDER_MAX_STALL_CURRENT);

        RobotConfigs.getInstance().addConfigurable(Constants.Shooter.FEEDER_VELOCITY_CONTROLLER_CONFIGURABLE_LABEL, m_spark);
    }

    public void addToShuffleboard() {
        ShuffleboardTab tab = Shuffleboard.getTab(Constants.Shooter.TAB_NAME);
        tab.addNumber("Feeder Current", m_spark::getOutputCurrent);
        tab.addNumber("Feeder Velocity", this::getEncoderVelocity);
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

    public void tunePeriodic() {
        m_spark.runPID();
    }

}
