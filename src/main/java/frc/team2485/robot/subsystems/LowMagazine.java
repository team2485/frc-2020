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

    /**
     * the number of balls currently contained in the low belt
     */
    private int m_numBalls;

    private boolean m_entranceIRLastVal, m_transferIRLastVal, m_exitIRLastVal;

    /**
     * Low magazine subystem, controlling the intake rollers and low belt.
     */
    public LowMagazine() {
        m_talon = new PIDTalonSRX(Constants.Magazine.TALON_LOW_PORT, ControlMode.Current);

        m_entranceIR = new DigitalInput(Constants.Magazine.ENTRANCE_IR_PORT);
        m_transferIR = new DigitalInput(Constants.Magazine.TRANSFER_IR_PORT);

        m_numBalls = 0;

        m_entranceIRLastVal = false;
    }

    /**
     * Sets talon to a specific PWM
     * @param pwm PWM to set the talon to
     */
    public void setPWM(double pwm) {
        m_talon.set(pwm);
    }

    /**
     * Sets the talon to a specific current using PID control
     * @param current current to set the PID to
     */
    public void setCurrent(double current) {
        m_talon.runPID(current);
    }

    /**
     *
     * @return belt encoder position
     */
    public double getEncoderPosition() {
        return m_talon.getSensorCollection().getQuadraturePosition();
    }

    /**
     *
     * @return belt encoder velocity
     */
    public double getEncoderVelocity() {
        return Deadband.deadband(m_talon.getSensorCollection().getQuadraturePosition(), Constants.Magazine.ENCODER_VELOCITY_DEADBAND);
    }

    /**
     *
     * @return boolean value of beam break sensor at start of low belt
     */
    public boolean getEntranceIR() {
        return m_entranceIR.get();
    }

    /**
     *
     * @return value of beam break sensor at intersection of low and high belts
     */
    public boolean getTransferIR() {
        return m_transferIR.get();
    }

    /**
     *
     * @return boolean number of balls currently in the high belt
     */
    public int getNumBalls() {
        return m_numBalls;
    }

    /**
     * Periodic method updating the number of balls in the low belt using beam break sensors.
     */
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
