package frc.team2485.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.wpilibj.smartdashboard.SendableRegistry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team2485.WarlordsLib.Limelight;
import frc.team2485.WarlordsLib.motorcontrol.PIDTalonSRX;
import frc.team2485.WarlordsLib.sensors.TalonSRXEncoder;
import frc.team2485.robot.Constants;

public class Turret extends SubsystemBase {

    private PIDTalonSRX m_talon;

    private Limelight m_limelight;

    private TalonSRXEncoder m_encoder;

    public Turret() {
        m_talon = new PIDTalonSRX(Constants.Turret.TALON_PORT, ControlMode.MotionProfile);

        m_talon.enableVoltageCompensation();

        m_encoder = new TalonSRXEncoder(m_talon, TalonSRXEncoder.TalonSRXEncoderType.ABSOLUTE, Constants.Turret.ENCODER_CPR);
        m_encoder.setDistancePerRevolution(360);

        m_limelight = new Limelight();

        SendableRegistry.add(m_talon, "Turret Talon");
        SmartDashboard.putData(this);
    }

    public void setPWM(double pwm) {
        m_talon.set(pwm);
    }

    public void setPositionPID(double position) {
        m_talon.runPID(position);
    }

    /**
     * Reset the encoder position to a given position
     * @param position position to set to.
     */
    public void setEncoderPosition(double position) {
        m_encoder.resetPosition(position);
    }

    /**
     * Get turret encoder position
     * @return encoder position in radians
     */
    public double getEncoderPosition() {
        return m_encoder.getPosition();
    }

    public Limelight getLimelight() {
        return this.m_limelight;
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Turret Encoder Position", this.getEncoderPosition());
    }
}
