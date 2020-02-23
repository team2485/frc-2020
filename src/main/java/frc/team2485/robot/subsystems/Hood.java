package frc.team2485.robot.subsystems;

import com.revrobotics.CANDigitalInput;
import com.revrobotics.CANEncoder;
import com.revrobotics.ControlType;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableRegistry;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team2485.WarlordsLib.motorcontrol.PIDSparkMax;
import frc.team2485.WarlordsLib.motorcontrol.WL_SparkMax;
import frc.team2485.WarlordsLib.robotConfigs.RobotConfigs;
import frc.team2485.robot.Constants;


public class Hood extends SubsystemBase {
    private WL_SparkMax m_spark;
    private CANEncoder m_hoodEncoder;

    public Hood(CANEncoder hoodEncoder) {
        this.m_spark = new WL_SparkMax(Constants.Shooter.SPARK_HOOD_PORT);
        this.m_spark.enableVoltageCompensation(12);

        this.m_spark.getForwardLimitSwitch(CANDigitalInput.LimitSwitchPolarity.kNormallyClosed).enableLimitSwitch(true);
        this.m_spark.getReverseLimitSwitch(CANDigitalInput.LimitSwitchPolarity.kNormallyClosed).enableLimitSwitch(true);

        this.m_hoodEncoder = hoodEncoder;
        this.m_hoodEncoder.setPositionConversionFactor(Constants.Shooter.HOOD_DISTANCE_PER_REVOLUTION);

//        SendableRegistry.add(m_spark, "Hood Spark");
//        RobotConfigs.getInstance().addConfigurable("hoodSpark", m_spark);

        ShuffleboardTab tab = Shuffleboard.getTab("Shooter");
        tab.addNumber("Hood Encoder Position", this::getEncoderPosition);
    }

    public void setPWM(double pwm) {
        m_spark.set(pwm);
    }

    public double getEncoderPosition() {
        return m_hoodEncoder.getPosition();
    }

    public boolean getTopLimitSwitch() {
        return m_spark.getReverseLimitSwitch(CANDigitalInput.LimitSwitchPolarity.kNormallyClosed).get();
    }

    public boolean getBottomLimitSwitch() {
        return m_spark.getForwardLimitSwitch(CANDigitalInput.LimitSwitchPolarity.kNormallyClosed).get();
    }

    @Override
    public void periodic() {
        if (getBottomLimitSwitch()) {
            this.m_hoodEncoder.setPosition(Constants.Shooter.HOOD_BOTTOM_POSITION);
        } else if (getTopLimitSwitch()) {
            this.m_hoodEncoder.setPosition(Constants.Shooter.HOOD_TOP_POSITION);
        }
    }
}
