package frc.team2485.robot.subsystems;

import edu.wpi.first.wpilibj.Counter;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team2485.WarlordsLib.Debounce;
import frc.team2485.robot.Constants;

import java.util.function.DoubleSupplier;

public class BallCounter extends SubsystemBase {

    private DigitalInput m_entranceIR, m_transferIR, m_exitIR;

    private int m_nBallsLow, m_nBallsHigh;

    private DoubleSupplier m_lowEncoderVelocity, m_highEncoderVelocity;

    private boolean m_entranceLastVal, m_transferLastVal, m_exitLastVal;

    private Debounce m_entranceDebounce, m_transferDebounce, m_exitDebounce;

    private boolean m_entranceVal, m_transferVal, m_exitVal;

    public BallCounter(DoubleSupplier lowEncoderVelocity, DoubleSupplier highEncoderVelocity) {
        m_lowEncoderVelocity = lowEncoderVelocity;
        m_highEncoderVelocity = highEncoderVelocity;

        m_entranceIR = new DigitalInput(Constants.Magazine.ENTRANCE_IR_PORT);
        m_transferIR = new DigitalInput(Constants.Magazine.TRANSFER_IR_PORT);
        m_exitIR = new DigitalInput(Constants.Magazine.EXIT_IR_PORT);

        m_entranceDebounce = new Debounce(m_entranceIR.get(), Constants.Magazine.MAX_DEBOUNCE_TIME);
        m_transferDebounce = new Debounce(m_transferIR.get(), Constants.Magazine.MAX_DEBOUNCE_TIME);
        m_exitDebounce = new Debounce(m_exitIR.get(), Constants.Magazine.MAX_DEBOUNCE_TIME);

        this.addToShuffleboard();
    }

    public void addToShuffleboard() {
        ShuffleboardTab tab = Shuffleboard.getTab(Constants.Magazine.TAB_NAME);
        tab.addNumber("Low Mag Ball Count", this::getNumBallsLow);
        tab.addNumber("High Mag Ball Count", this::getNumBallsHigh);
        tab.addBoolean("Entrance IR", this::entranceIRHasBall);
        tab.addBoolean("Transfer IR", this::transferIRHasBall);
        tab.addBoolean("Exit IR", this::exitIRHasBall);
    }

    public int getNumBallsLow() {
        return this.m_nBallsLow;
    }

    public int getNumBallsHigh() {
        return this.m_nBallsHigh;
    }

    /**
     * Reset ball count in low magazine
     * @param num number of balls counted
     */
    public void setNumBallsLow(int num) {
        this.m_nBallsLow = num;
    }

    /**
     * Reset ball count in high magazine
     * @param num number of balls counted
     */
    public void setNumBallsHigh(int num) {
        this.m_nBallsHigh = num;
    }

    /**
     * True if ball is in IR
     */
    public boolean entranceIRHasBall() {
        return m_entranceVal;
    }

    /**
     * True if ball is in IR
     */
    public boolean transferIRHasBall() {
        return m_transferVal;
    }

    /**
     * True if ball is in IR
     */
    public boolean exitIRHasBall() {
        return m_exitVal;
    }

    @Override
    public void periodic() {

        m_entranceVal = !m_entranceDebounce.getNextValue(m_entranceIR.get());
        m_transferVal = !m_transferDebounce.getNextValue(m_transferIR.get());
        m_exitVal = !m_exitDebounce.getNextValue(m_exitIR.get());

        if (!m_entranceVal && m_entranceLastVal) {
            if (m_lowEncoderVelocity.getAsDouble() < 0) {
                m_nBallsLow++;
            } else if (m_lowEncoderVelocity.getAsDouble() > 0) {
                m_nBallsLow--;
            }
        }

        if (!m_transferVal && m_transferLastVal) {
            if (m_highEncoderVelocity.getAsDouble() < 0) {
                m_nBallsHigh++;
            } else if (m_highEncoderVelocity.getAsDouble() > 0) {
                m_nBallsHigh--;
            }

            if (m_lowEncoderVelocity.getAsDouble() < 0) {
                m_nBallsLow--;
            } else if (m_lowEncoderVelocity.getAsDouble() > 0) {
                m_nBallsLow++;
            }
        }

        if (!m_exitVal && m_exitLastVal) {
            if (m_highEncoderVelocity.getAsDouble() < 0) {
                m_nBallsHigh--;
            } else if (m_highEncoderVelocity.getAsDouble() > 0) {
                m_nBallsHigh++;
            }
        }

        if (m_nBallsHigh < 0) {
            m_nBallsHigh = 0;
        }

        if (m_nBallsLow < 0) {
            m_nBallsLow = 0;
        }

        m_entranceLastVal = m_entranceVal;
        m_transferLastVal = m_transferVal;
        m_exitLastVal = m_exitVal;
    }
}
