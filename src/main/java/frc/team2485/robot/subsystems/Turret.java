package frc.team2485.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.revrobotics.CANError;
import edu.wpi.first.wpilibj.smartdashboard.SendableRegistry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team2485.WarlordsLib.motorcontrol.PIDTalonSRX;
import frc.team2485.WarlordsLib.sensors.TalonSRXEncoderWrapper;
import frc.team2485.robot.Constants;

public class Turret extends SubsystemBase {

    private PIDTalonSRX m_talon;

    private TalonSRXEncoderWrapper m_encoder;

    public Turret() {
        m_talon = new PIDTalonSRX(Constants.TurretConstants.TALON_PORT, ControlMode.MotionProfile);

        m_encoder = new TalonSRXEncoderWrapper(m_talon, TalonSRXEncoderWrapper.TalonSRXEncoderType.ABSOLUTE, Constants.TurretConstants.ENCODER_CPR);

        SendableRegistry.add(m_talon, "Turret Talon");

        SmartDashboard.putData(this);
    }

    public void setPWM(double pwm) {
        m_talon.set(pwm);
    }

    public void setPosition(double position) {
        
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Turret Encoder Position", m_encoder.getPosition());
    }
}
