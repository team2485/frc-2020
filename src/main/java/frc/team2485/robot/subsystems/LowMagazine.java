package frc.team2485.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team2485.WarlordsLib.motorcontrol.PIDTalonSRX;
import frc.team2485.WarlordsLib.oi.Deadband;
import frc.team2485.robot.Constants;

public class LowMagazine extends SubsystemBase {

    private PIDTalonSRX m_talon;

    private DigitalInput m_entranceIR, m_transferIR;

    private int m_numBalls;

    private boolean m_entranceIRLastVal, m_transferIRLastVal, m_exitIRLastVal;

    public LowMagazine() {
        m_talon = new PIDTalonSRX(Constants.Magazine.TALON_LOW_PORT, ControlMode.Current);

        m_entranceIR = new DigitalInput(Constants.Magazine.ENTRANCE_IR_PORT);
        m_transferIR = new DigitalInput(Constants.Magazine.TRANSFER_IR_PORT);

        m_numBalls = 0;

        m_entranceIRLastVal = false;
    }

    public void setPWM(double pwm) {
        m_talon.set(pwm);
    }

    public void setCurrent(double current) {
        m_talon.runPID(current);
    }

    public double getEncoderPosition() {
        return Deadband.deadband(m_talon.getSensorCollection().getQuadraturePosition(), Constants.Magazine.ENCODER_VELOCITY_DEADBAND);
    }

    public double getEncoderVelocity() {
        return m_talon.getSensorCollection().getQuadraturePosition();
    }

    public boolean getEntranceIR() {
        return m_entranceIR.get();
    }

    public boolean getTransferIR() {
        return m_transferIR.get();
    }

    public int getNumBalls() {
        return m_numBalls;
    }

    @Override
    public void periodic() {
        if (!getEntranceIR() && getEntranceIR() != m_entranceIRLastVal) {
            if (getEncoderVelocity() < 0) {
                m_numBalls++;
            } else if (getEncoderVelocity() > 0) {
                m_numBalls--;
            }
        }

        if (!getTransferIR() && getTransferIR() != m_transferIRLastVal) {
            if (getEncoderVelocity() < 0) {
                m_numBalls--;
            } else if (getEncoderVelocity() > 0) {
                m_numBalls++;
            }
        }

        m_entranceIRLastVal = getEntranceIR();
        m_transferIRLastVal = getTransferIR();
    }
}
