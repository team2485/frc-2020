package frc.team2485.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableRegistry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpiutil.math.MathUtil;
import frc.team2485.WarlordsLib.BufferZone;
import frc.team2485.WarlordsLib.Limelight;
import frc.team2485.WarlordsLib.PositionPIDSubsystem;
import frc.team2485.WarlordsLib.VelocityPIDSubsystem;
import frc.team2485.WarlordsLib.control.WL_PIDController;
import frc.team2485.WarlordsLib.motorcontrol.PIDTalonSRX;
import frc.team2485.WarlordsLib.robotConfigs.Configurable;
import frc.team2485.WarlordsLib.robotConfigs.LoadableConfigs;
import frc.team2485.WarlordsLib.robotConfigs.RobotConfigs;
import frc.team2485.WarlordsLib.robotConfigs.SavableConfigs;
import frc.team2485.robot.Constants;

public class Turret extends SubsystemBase implements VelocityPIDSubsystem, PositionPIDSubsystem, Configurable {

    private PIDTalonSRX m_talon;

    private WL_PIDController m_positionController;

    private Limelight m_limelight;

    private BufferZone m_buffer;

    private final double MIN_ANGLE, MAX_ANGLE, BUFFER_ZONE_SIZE;

    private double m_absoluteEncoderOffset;

    public Turret() {

        m_talon = new PIDTalonSRX(Constants.Turret.TALON_PORT, ControlMode.Velocity);
        m_talon.configNominalOutputForward(0);
        m_talon.configNominalOutputReverse(0);
        m_talon.configPeakOutputForward(1);
        m_talon.configPeakOutputReverse(-1);
        m_talon.enableVoltageCompensation();
        m_talon.setFeedbackDeviceType(FeedbackDevice.CTRE_MagEncoder_Relative);
        m_talon.setDistancePerPulse(360.0 / Constants.Turret.ENCODER_CPR); // convert to degrees
        m_talon.enableVoltageCompensation(Constants.NOMINAL_VOLTAGE);
        m_talon.setTolerance(Constants.Turret.TURRET_POSITION_TOLERANCE);
        m_talon.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyClosed);
        m_talon.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyClosed);

        m_absoluteEncoderOffset = RobotConfigs.getInstance().getDouble(Constants.Turret.ENCODER_OFFSET_CONFIGURABLE_LABEL, "encoderOffset", 0);

        m_positionController = new WL_PIDController();

        m_limelight = new Limelight();

        MIN_ANGLE = Constants.Turret.MIN_POSITION;
        MAX_ANGLE = Constants.Turret.MAX_POSITION;

        BUFFER_ZONE_SIZE = Constants.Turret.BUFFER_ZONE_SIZE;

        m_buffer = new BufferZone(Constants.Turret.MIN_VELOCITY, Constants.Turret.MAX_VELOCITY, Constants.Turret.MIN_POSITION, Constants.Turret.MAX_POSITION, Constants.Turret.BUFFER_ZONE_SIZE);

        RobotConfigs.getInstance().addConfigurable(Constants.Turret.VELOCITY_CONTROLLER_CONFIGURABLE_LABEL, m_talon);
        RobotConfigs.getInstance().addConfigurable(Constants.Turret.POSITION_CONTROLLER_CONFIGURABLE_LABEL, m_positionController);
        RobotConfigs.getInstance().addConfigurable(Constants.Turret.ENCODER_OFFSET_CONFIGURABLE_LABEL, this);

        this.addToShuffleboard();
    }

    public void addToShuffleboard() {
        SendableRegistry.add(m_talon, "Turret Talon");
        ShuffleboardTab tab = Shuffleboard.getTab(Constants.Turret.TAB_NAME);
        tab.add(this);
        tab.add("Velocity Controller", m_talon);
        tab.add("Position Controller", m_positionController);
        tab.addNumber("Encoder Position", this::getEncoderPosition);
        tab.addNumber("Encoder Velocity", this::getEncoderVelocity);
        tab.addNumber("Encoder Abs Position", this::getAbsoluteEncoderPosition);
        tab.addNumber("Encoder Offset", this::getEncoderOffset);
        tab.addNumber("Current", m_talon::getStatorCurrent);
    }

    /**
     * Set PWM of turret motor, clamped based on proximity to limits.
     * @param pwm pwm to set.
     */
    public void setClampedPWM(double pwm) {
        double output = 0;

        // Set the max pwm output based on proximity to limits
        if (pwm < 0) {
            output = Math.copySign(MathUtil.clamp(Math.abs(pwm), 0, Math.pow((MIN_ANGLE -  this.getEncoderPosition()) * (-1 / BUFFER_ZONE_SIZE), 2)), pwm);
        } else if (pwm > 0) {
            output = MathUtil.clamp(pwm, 0, Math.pow((MAX_ANGLE - this.getEncoderPosition()) * (1 / BUFFER_ZONE_SIZE), 2));
        }

        SmartDashboard.putNumber("Low Clamp", Math.pow((MIN_ANGLE -  this.getEncoderPosition()) * (-1 / BUFFER_ZONE_SIZE), 2));
        SmartDashboard.putNumber("High Clamp", Math.pow((MAX_ANGLE - this.getEncoderPosition()) * (1 / BUFFER_ZONE_SIZE), 2));

        m_talon.set(output);
    }

    /**
     * Sets PWM of turret motor, without hard stop checks.
     * @param pwm pwm value to set
     */
    public void setPWM(double pwm) {
        m_talon.set(pwm);
    }

    @Override
    public void runVelocityPID(double velocity) {
        // velocity / 10 to convert from per 100ms to 1 second
        m_talon.runPID(m_buffer.get(velocity, this.getEncoderPosition())/10.0);
    }

    public void runUnclampedVelocityPID(double velocity) {
        m_talon.runPID(velocity/10.0);
    }

    @Override
    public void runPositionPID(double position) {
        m_positionController.setSetpoint(MathUtil.clamp(position, MIN_ANGLE, MAX_ANGLE));
        this.runVelocityPID(m_positionController.calculate(this.getEncoderPosition()));
    }

    /**
     * Whether the turret's talon's pid is at target based on threhold.
     * @return true if pid is at target within threshold.
     */
    @Override
    public boolean atVelocitySetpoint() {
        return m_talon.atTarget();
    }

    @Override
    public boolean atPositionSetpoint() { return m_positionController.atSetpoint(); }

    /**
     * Get turret encoder position
     */
    @Override
    public double getEncoderPosition() {
        return m_talon.getEncoderPosition();
    }

    @Override
    public double getEncoderVelocity() { return m_talon.getEncoderVelocity() * 10; }

    public double getAbsoluteEncoderPosition() {
        return m_talon.getSensorCollection().getPulseWidthPosition() * m_talon.getConversionFactor();
    }

    public Limelight getLimelight() {
        return this.m_limelight;
    }

    public double getMinAngle() {
        return this.MIN_ANGLE;
    }

    public double getMaxAngle() {
        return this.MAX_ANGLE;
    }

    public void enableSoftLimits(boolean enable) {
        m_talon.configForwardSoftLimitEnable(enable);
        m_talon.configReverseSoftLimitEnable(enable);
    }

    public boolean getForwardLimitSwitch() {
        return !m_talon.getSensorCollection().isFwdLimitSwitchClosed();
    }

    public boolean getReverseLimitSwitch() {
        return !m_talon.getSensorCollection().isRevLimitSwitchClosed();
    }


    @Override
    public void resetPIDs() {
        m_talon.resetPID();
        m_positionController.reset();
    }

    /**
     * Reset the encoder position to a given position
     * @param position position to set to.
     */
    public void resetEncoderPosition(double position) {
        m_talon.setEncoderPosition(position);
        this.m_absoluteEncoderOffset = position - this.getAbsoluteEncoderPosition();
    }

    @Override
    public void periodic() {
        if (getReverseLimitSwitch()) {
            resetEncoderPosition(MIN_ANGLE);
        } else if (getForwardLimitSwitch()) {
            resetEncoderPosition(MAX_ANGLE);
        }
    }

    private void setEncoderOffset(double offset) {
        m_talon.setEncoderPosition(this.getAbsoluteEncoderPosition() + offset);
        this.m_absoluteEncoderOffset = offset;
    }

    private double getEncoderOffset() {
        return this.m_absoluteEncoderOffset;
    }

    @Override
    public void tunePeriodic(int layer) {
        System.out.println(layer == 0);
        if (layer == 0) {
            m_talon.runPID();
        } else if (layer == 1) {
            this.runVelocityPID(m_positionController.calculate(this.getEncoderPosition()));
        }
    }

    @Override
    public void loadConfigs(LoadableConfigs configs) {
        setEncoderOffset(configs.getDouble("encoderOffset", 0));
    }

    @Override
    public void saveConfigs(SavableConfigs configs) {
        configs.put("encoderOffset", m_absoluteEncoderOffset);
    }
}
