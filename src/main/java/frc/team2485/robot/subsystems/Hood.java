package frc.team2485.robot.subsystems;

import com.revrobotics.CANDigitalInput;
import com.revrobotics.CANEncoder;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpiutil.math.MathUtil;
import frc.team2485.WarlordsLib.PositionPIDSubsystem;
import frc.team2485.WarlordsLib.VelocityPIDSubsystem;
import frc.team2485.WarlordsLib.control.WL_PIDController;
import frc.team2485.WarlordsLib.motorcontrol.WL_SparkMax;
import frc.team2485.WarlordsLib.robotConfigs.RobotConfigs;
import frc.team2485.robot.Constants;


public class Hood extends SubsystemBase implements PositionPIDSubsystem, VelocityPIDSubsystem {

    private WL_SparkMax m_spark;
    private CANEncoder m_hoodEncoder;

    private WL_PIDController m_velocityController;

    private WL_PIDController m_positionController;

    public Hood(CANEncoder hoodEncoder) {
        this.m_spark = new WL_SparkMax(Constants.Hood.SPARK_PORT);
        this.m_spark.enableVoltageCompensation(Constants.NOMINAL_VOLTAGE);

        this.m_spark.getForwardLimitSwitch(CANDigitalInput.LimitSwitchPolarity.kNormallyClosed).enableLimitSwitch(true);
        this.m_spark.getReverseLimitSwitch(CANDigitalInput.LimitSwitchPolarity.kNormallyClosed).enableLimitSwitch(true);

        this.m_hoodEncoder = hoodEncoder;
        this.m_hoodEncoder.setPositionConversionFactor(Constants.Hood.DISTANCE_PER_REVOLUTION);
        this.m_hoodEncoder.setVelocityConversionFactor(Constants.Hood.DISTANCE_PER_REVOLUTION);

        this.m_velocityController = new WL_PIDController();
        this.m_positionController = new WL_PIDController();

        RobotConfigs.getInstance().addConfigurable(Constants.Hood.HOOD_VELOCITY_CONTROLLER_CONFIGURABLE_LABEL, m_velocityController);
        RobotConfigs.getInstance().addConfigurable(Constants.Hood.HOOD_POSITION_CONTROLLER_CONFIGURABLE_LABEL, m_positionController);

        this.addToShuffleboard();
    }

    public void addToShuffleboard() {
        ShuffleboardTab tab = Shuffleboard.getTab(Constants.Hood.TAB_NAME);
        tab.add(m_spark);
        tab.add("Hood Velocity Ctrl", m_velocityController);
        tab.add("Hood Position Ctrl", m_positionController);
        tab.addNumber("Hood Encoder Velocity", this::getEncoderVelocity);
        tab.addNumber("Hood Encoder Position", this::getEncoderPosition);
        tab.addNumber("Hood Current", m_spark::getOutputCurrent);
    }

    public void setPWM(double pwm) {
        m_spark.set(pwm);
    }

    @Override
    public void runVelocityPID(double velocity) {
        this.setPWM(m_velocityController.calculate(this.getEncoderVelocity(), velocity));
    }

    @Override
    public void runPositionPID(double position) {
        runVelocityPID(m_positionController.calculate(this.getEncoderPosition(), MathUtil.clamp(position, Constants.Hood.HOOD_TOP_POSITION_DEG, Constants.Hood.HOOD_BOTTOM_POSITION_DEG)));
    }

    @Override
    public boolean atVelocitySetpoint() {
        return m_velocityController.atSetpoint();
    }

    @Override
    public boolean atPositionSetpoint() {
        return m_positionController.atSetpoint();
    }

    @Override
    public double getEncoderVelocity() {
        return m_hoodEncoder.getVelocity();
    }

    @Override
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
        if (getBottomLimitSwitch()) {
            this.m_hoodEncoder.setPosition(Constants.Hood.HOOD_BOTTOM_POSITION_DEG);
        }
        else
            if (getTopLimitSwitch()) {
            this.m_hoodEncoder.setPosition(Constants.Hood.HOOD_TOP_POSITION_DEG);
        }
    }


    /**
     * Should run periodically and run the motor to tune
     */
    @Override
    public void tunePeriodic(int layer) {
        if (layer == 0) {
            m_spark.set(m_velocityController.calculate(this.getEncoderVelocity()));
        } else if (layer == 1) {
            runVelocityPID(m_positionController.calculate(this.getEncoderPosition()));
        }
    }

}
