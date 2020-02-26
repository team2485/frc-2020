package frc.team2485.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableRegistry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpiutil.math.MathUtil;
import frc.team2485.WarlordsLib.Limelight;
import frc.team2485.WarlordsLib.Tunable;
import frc.team2485.WarlordsLib.motorcontrol.PIDTalonSRX;
import frc.team2485.WarlordsLib.robotConfigs.RobotConfigs;
import frc.team2485.WarlordsLib.sensors.TalonSRXEncoder;
import frc.team2485.robot.Constants;

public class Turret extends SubsystemBase implements Tunable {

    private PIDTalonSRX m_talon;

    private Limelight m_limelight;

    private final double MIN_ANGLE, MAX_ANGLE, BUFFER_ZONE_SIZE;

    public Turret() {

        m_talon = new PIDTalonSRX(Constants.Turret.TALON_PORT, ControlMode.Position);
        m_talon.configNominalOutputForward(0);
        m_talon.configNominalOutputReverse(0);
        m_talon.configPeakOutputForward(1);
        m_talon.configPeakOutputReverse(-1);
        m_talon.enableVoltageCompensation();
        m_talon.setFeedbackDeviceType(FeedbackDevice.CTRE_MagEncoder_Relative);
        m_talon.setSelectedSensorPosition(m_talon.getSensorCollection().getPulseWidthPosition());
        m_talon.setDistancePerPulse(360.0 / Constants.Turret.ENCODER_CPR); // convert to degrees
        m_talon.enableVoltageCompensation(Constants.NOMINAL_VOLTAGE);
        m_talon.setTolerance(Constants.Turret.TURRET_PID_TOLERANCE);
        m_talon.configReverseSoftLimitThreshold((int) (Constants.Turret.MIN_POSITION * Constants.Turret.ENCODER_CPR / 360));
        m_talon.configReverseSoftLimitEnable(true);
        m_talon.configForwardSoftLimitThreshold((int) (Constants.Turret.MAX_POSITION * Constants.Turret.ENCODER_CPR / 360));
        m_talon.configForwardSoftLimitEnable(true);

        m_limelight = new Limelight();

        MIN_ANGLE = Constants.Turret.MIN_POSITION;
        MAX_ANGLE = Constants.Turret.MAX_POSITION;

        BUFFER_ZONE_SIZE = Constants.Turret.BUFFER_ZONE_SIZE;

        RobotConfigs.getInstance().addConfigurable(Constants.Turret.POSITION_CONTROLLER_CONFIGURABLE_LABEL, m_talon);

        this.addToShuffleboard();
    }

    public void addToShuffleboard() {
        SendableRegistry.add(m_talon, "Turret Talon");
        ShuffleboardTab tab = Shuffleboard.getTab(Constants.Turret.TAB_NAME);
        tab.add(this);
        tab.addNumber("Encoder Position", this::getEncoderPosition);
        tab.addNumber("Current", m_talon::getStatorCurrent);
    }

    /**
     * Set PWM of turret motor, clamped based on proximity to limits.
     * @param pwm pwm to set.
     */
    public void setPWM(double pwm) {
        double output = 0;

        // Set the max pwm output based on proximity to limits
        if (pwm < 0) {
            output = Math.copySign(MathUtil.clamp(Math.abs(pwm), 0, Math.pow((MIN_ANGLE -  this.getEncoderPosition()) * (-1 / BUFFER_ZONE_SIZE), 2)), pwm);
        } else if (pwm > 0) {
            output = MathUtil.clamp(pwm, 0, Math.pow((MAX_ANGLE - this.getEncoderPosition()) * (1 / BUFFER_ZONE_SIZE), 2));
        }

        m_talon.set(output);
    }

    /**
     * Sets PWM of turret motor, without hard stop checks.
     * DON'T USE THIS UNLESS YOU'RE ABSOLUTELY SURE WHAT YOU'RE DOING!
     * @param pwm pwm value to set
     */
    public void setUnclampedPWM(double pwm) {
        m_talon.set(pwm);
    }

    /**
     * Reset the encoder position to a given position
     * @param position position to set to.
     */
    public void resetEncoderPosition(double position) {
        m_talon.setEncoderPosition(position);
        m_talon.getSensorCollection().setPulseWidthPosition((int) (position * Constants.Turret.ENCODER_CPR / 360.0), 0);
    }

    /**
     * Set a position using the turret's talon's pid.
     * @param setpoint position to set (degrees)
     * @return true if pid is at target within threshold.
     */
    public boolean runPID(double setpoint) {
        m_talon.runPID(MathUtil.clamp(setpoint, this.MIN_ANGLE, this.MAX_ANGLE));
        return m_talon.atTarget();
    }

    /**
     * Whether the turret's talon's pid is at target based on threhold.
     * @return true if pid is at target within threshold.
     */
    public boolean atPIDTarget() {
        return m_talon.atTarget();
    }

    /**
     * Get turret encoder position
     * @return encoder position in radians
     */
    public double getEncoderPosition() {
        return m_talon.getEncoderPosition();
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

    public void resetPID() {
        this.m_talon.resetPID();
    }

    public void enableSoftLimits(boolean enable) {
        m_talon.configForwardSoftLimitEnable(enable);
        m_talon.configReverseSoftLimitEnable(enable);
    }

    @Override
    public void periodic() {
    }

    @Override
    public void tunePeriodic() {
        m_talon.runPID();
    }
}
