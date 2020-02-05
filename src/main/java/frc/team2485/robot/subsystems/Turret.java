package frc.team2485.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import edu.wpi.first.wpilibj.smartdashboard.SendableRegistry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpiutil.math.MathUtil;
import frc.team2485.WarlordsLib.Limelight;
import frc.team2485.WarlordsLib.motorcontrol.PIDTalonSRX;
import frc.team2485.WarlordsLib.robotConfigs.RobotConfigs;
import frc.team2485.robot.Constants;

public class Turret extends SubsystemBase {

    private PIDTalonSRX m_talon;

    private Limelight m_limelight;

    private double m_minAngle, m_maxAngle, m_bufferZoneSize;

    public Turret() {

        m_talon = new PIDTalonSRX(Constants.Turret.TALON_PORT, ControlMode.Position);
        m_talon.enableVoltageCompensation();
        m_talon.setFeedbackDeviceType(FeedbackDevice.CTRE_MagEncoder_Relative);
        m_talon.setSelectedSensorPosition(m_talon.getSensorCollection().getPulseWidthPosition());
        m_talon.setConversionFactor(360.0 / Constants.Turret.ENCODER_CPR);
        m_talon.setTolerance(Constants.Turret.TURRET_PID_TOLERANCE);

        m_limelight = new Limelight();

        m_minAngle = Constants.Turret.MIN_POSITION;
        m_maxAngle = Constants.Turret.MAX_POSITION;

        m_bufferZoneSize = Constants.Turret.BUFFER_ZONE_SIZE;

        RobotConfigs.getInstance().addConfigurable("Turret TalonSRX PID", m_talon);

        SendableRegistry.add(m_talon, "Turret Talon");

        SmartDashboard.putData(this);
    }

    public void setPWM(double pwm) {
        double output = 0;

        // Set the max pwm output based on proximity to limits
        if (pwm < 0) {
            output = Math.copySign(MathUtil.clamp(Math.abs(pwm), 0, (m_minAngle -  this.getEncoderPosition()) * (-1 / m_bufferZoneSize)), pwm);
        } else if (pwm > 0) {
            output = MathUtil.clamp(pwm, 0, (m_maxAngle - this.getEncoderPosition()) * (1 / m_bufferZoneSize));
        }

        m_talon.set(output);
    }

    /**
     * Reset the encoder position to a given position
     * @param position position to set to.
     */
    public void setEncoderPosition(double position) {
        m_talon.setEncoderPosition(position);
    }

    public boolean runPID(double setpoint) {
        m_talon.runPID(MathUtil.clamp(setpoint, this.m_minAngle, this.m_maxAngle));
        return m_talon.atTarget();
    }

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
        return this.m_minAngle;
    }

    public double getMaxAngle() {
        return this.m_maxAngle;
    }

    public void resetPID() {
        this.m_talon.resetPID();
    }


    @Override
    public void periodic() {
        SmartDashboard.putNumber("Turret Encoder Position", (this.getEncoderPosition()));
        SmartDashboard.putNumber("Turret Supply Currentokai", m_talon.getSupplyCurrent());

        SmartDashboard.putNumber("Turret Stator Currentokai", m_talon.getStatorCurrent());
    }
}
