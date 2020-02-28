package frc.team2485.robot.subsystems;

import com.revrobotics.CANDigitalInput;
import com.revrobotics.CANEncoder;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team2485.WarlordsLib.motorcontrol.WL_SparkMax;
import frc.team2485.robot.Constants;


public class Hood extends SubsystemBase {

    private WL_SparkMax m_spark;
    private CANEncoder m_hoodEncoder;

    public Hood(CANEncoder hoodEncoder) {
        this.m_spark = new WL_SparkMax(Constants.Hood.SPARK_PORT);
        this.m_spark.enableVoltageCompensation(Constants.NOMINAL_VOLTAGE);

        this.m_spark.getForwardLimitSwitch(CANDigitalInput.LimitSwitchPolarity.kNormallyClosed).enableLimitSwitch(true);
        this.m_spark.getReverseLimitSwitch(CANDigitalInput.LimitSwitchPolarity.kNormallyClosed).enableLimitSwitch(true);

        this.m_hoodEncoder = hoodEncoder;
        this.m_hoodEncoder.setPositionConversionFactor(Constants.Hood.DISTANCE_PER_REVOLUTION);

        this.addToShuffleboard();
    }

    public void addToShuffleboard() {
        ShuffleboardTab tab = Shuffleboard.getTab(Constants.Hood.TAB_NAME);
        tab.addNumber("Hood Encoder Position", this::getEncoderPosition);
    }

    public void setPWM(double pwm) {
        m_spark.set(pwm);
    }

    public double getEncoderPosition() {
        return m_hoodEncoder.getPosition();
    }

    /**
     * Returns the hood position relative to the horizontal in radians.
     * @return position in radians
     */
    public double getHoodTheta() {
        return Math.toRadians(90 - getEncoderPosition());
    }

    public boolean getTopLimitSwitch() {
        return m_spark.getReverseLimitSwitch(CANDigitalInput.LimitSwitchPolarity.kNormallyClosed).get();
    }

    public boolean getBottomLimitSwitch() {
        return m_spark.getForwardLimitSwitch(CANDigitalInput.LimitSwitchPolarity.kNormallyClosed).get();
    }

    @Override
    public void periodic() {
//        if (getBottomLimitSwitch()) {
//            this.m_hoodEncoder.setPosition(Constants.Shooter.HOOD_BOTTOM_POSITION_DEG);
//        } else
            if (getTopLimitSwitch()) {
            this.m_hoodEncoder.setPosition(Constants.Hood.HOOD_TOP_POSITION_DEG);
        }
    }
}
