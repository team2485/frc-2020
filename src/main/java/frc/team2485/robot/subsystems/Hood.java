package frc.team2485.robot.subsystems;

import com.revrobotics.CANDigitalInput;
import com.revrobotics.CANEncoder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpiutil.math.MathUtil;
import frc.team2485.WarlordsLib.BufferZone;
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

    private BufferZone m_velocityBuffer;

    private boolean m_isZeroed = false;

    public Hood(CANEncoder hoodEncoder) {
        this.m_spark = new WL_SparkMax(Constants.Hood.SPARK_PORT);
        this.m_spark.enableVoltageCompensation(Constants.NOMINAL_VOLTAGE);

        m_spark.setInverted(true);

        this.m_spark.getForwardLimitSwitch(CANDigitalInput.LimitSwitchPolarity.kNormallyClosed).enableLimitSwitch(true);
        this.m_spark.getReverseLimitSwitch(CANDigitalInput.LimitSwitchPolarity.kNormallyClosed).enableLimitSwitch(true);

        this.m_spark.getEncoder().setPositionConversionFactor(Constants.Hood.LEAD_SCREW_GEAR_RATIO);
        this.m_spark.getEncoder().setVelocityConversionFactor(Constants.Hood.LEAD_SCREW_GEAR_RATIO);

        this.m_hoodEncoder = hoodEncoder;
        hoodEncoder.setInverted(true);
        this.m_hoodEncoder.setPositionConversionFactor(Constants.Hood.DISTANCE_PER_REVOLUTION);
        this.m_hoodEncoder.setVelocityConversionFactor(Constants.Hood.DISTANCE_PER_REVOLUTION / 60);

        this.m_velocityController = new WL_PIDController();
        this.m_positionController = new WL_PIDController();

        this.m_positionController.setTolerance(Constants.Hood.POSITION_THRESHOLD);

        m_velocityBuffer = new BufferZone(-Constants.Hood.HOOD_MAX_VELOCITY, Constants.Hood.HOOD_MAX_VELOCITY,
                Constants.Hood.BOTTOM_POSITION_DEG, Constants.Hood.TOP_POSITION_DEG, Constants.Hood.BUFFER_ZONE_SIZE);

        RobotConfigs.getInstance().addConfigurable(Constants.Hood.HOOD_VELOCITY_CONTROLLER_CONFIGURABLE_LABEL, m_velocityController);
        RobotConfigs.getInstance().addConfigurable(Constants.Hood.HOOD_POSITION_CONTROLLER_CONFIGURABLE_LABEL, m_positionController);

        this.addToShuffleboard();
    }

    public void addToShuffleboard() {
        ShuffleboardTab tab = Shuffleboard.getTab(Constants.Hood.TAB_NAME);
        if (Constants.TUNE_MODE) {
            tab.add(m_spark);
            tab.add("Hood Velocity Ctrl", m_velocityController);
            tab.add("Hood Position Ctrl", m_positionController);
            tab.addNumber("Hood Encoder Velocity", this::getEncoderVelocity);
            tab.addNumber("Hood Neo Encoder Velocity", this::getNeoEncoderVelocity);
            tab.addNumber("Hood Current", m_spark::getOutputCurrent);
        }
        tab.addNumber("Hood Encoder Position", this::getEncoderPosition);
        tab.addBoolean("Hood at Target", this::atPositionSetpoint);
    }

    public void setPWM(double pwm) {
        m_spark.set(pwm);
    }

    @Override
    public void resetPIDs() {
        m_velocityController.reset();
        m_positionController.reset();
    }

    /**
     * Run velocity on the NEO 550
     * @param velocity in RPM
     */
    @Override
    public void runVelocityPID(double velocity) {
//        this.setPWM(m_velocityController.calculate(this.getEncoderVelocity(), MathUtil.clamp(velocity, Constants.Hood.HOOD_MIN_VELOCITY, Constants.Hood.HOOD_MAX_VELOCITY)));
        this.setPWM(m_velocityController.calculate(this.getEncoderVelocity(), m_velocityBuffer.get(velocity, getEncoderPosition())));
    }

    public void runUnclampedVelocityPID(double velocity) {
//        this.setPWM(m_velocityController.calculate(this.getEncoderVelocity(), MathUtil.clamp(velocity, Constants.Hood.HOOD_MIN_VELOCITY, Constants.Hood.HOOD_MAX_VELOCITY)));
        this.setPWM(m_velocityController.calculate(this.getEncoderVelocity(), velocity));
    }

    @Override
    public boolean atVelocitySetpoint() {
        return m_velocityController.atSetpoint();
    }

    @Override
    public double getEncoderVelocity() {
        return m_spark.getEncoder().getVelocity();
    }

    /**
     * Returns velocity of the NEO 550 driving the lead screw
     * @return velocity in RPM
     */
    public double getNeoEncoderVelocity() {
        return m_spark.getEncoder().getVelocity();
    }

    /**
     * Runs position PID in degrees from the normal
     * @param position in degrees
     */
    @Override
    public void runPositionPID(double position) {
        runVelocityPID(m_positionController.calculate(this.getEncoderPosition(), MathUtil.clamp(position, Constants.Hood.TOP_POSITION_DEG, Constants.Hood.BOTTOM_POSITION_DEG)));
    }

    @Override
    public boolean atPositionSetpoint() {
        return m_positionController.atSetpoint();
    }

    @Override
    public double getEncoderPosition() {
        return m_hoodEncoder.getPosition();
    }

    public boolean getReverseLimitSwitch() {
        return m_spark.getReverseLimitSwitch(CANDigitalInput.LimitSwitchPolarity.kNormallyClosed).get();
    }

    public boolean getForwardLimitSwitch() {
        return m_spark.getForwardLimitSwitch(CANDigitalInput.LimitSwitchPolarity.kNormallyClosed).get();
    }

    public void resetPID() {
        m_velocityController.reset();
        m_positionController.reset();
    }

    public void setEncoderPosition(double position) {
        m_hoodEncoder.setPosition(position);
    }

    public void forceZero() {
        if (getForwardLimitSwitch()) {
            this.m_hoodEncoder.setPosition(Constants.Hood.TOP_POSITION_DEG);
            m_isZeroed = true;
        } else if (getReverseLimitSwitch()) {
            this.m_hoodEncoder.setPosition(Constants.Hood.BOTTOM_POSITION_DEG);
            m_isZeroed = true;
        }
    }

    @Override
    public void periodic() {
        if (getForwardLimitSwitch()) {
            this.m_hoodEncoder.setPosition(Constants.Hood.TOP_POSITION_DEG);
            m_isZeroed = true;
        } else if (getReverseLimitSwitch()) {
            this.m_hoodEncoder.setPosition(Constants.Hood.BOTTOM_POSITION_DEG);
            m_isZeroed = true;
        }

        if (!m_isZeroed) {
            DriverStation.reportWarning("Hood Encoder Not Zeroed!", false);
        }
    }

    /**
     * Should run periodically and run the motor to tune
     */
    @Override
    public void tunePeriodic(int layer) {
        if (layer == 0) {
            setPWM(m_velocityController.calculate(this.getEncoderVelocity()));
        } else if (layer == 1) {
            runVelocityPID(m_positionController.calculate(this.getEncoderPosition()));
        }
    }
}
