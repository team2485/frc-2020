package frc.team2485.robot.subsystems;

import edu.wpi.first.wpilibj.Counter;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team2485.robot.Constants;

import java.util.function.DoubleSupplier;

public class BallCounter extends SubsystemBase {

    private DigitalInput m_entranceIR, m_transferIR, m_exitIR;

    private Counter m_entranceCounter, m_transferCounter, m_exitCounter;

    private int m_numBallsLow, m_numBallsHigh;

    private DoubleSupplier m_lowEncoderVelocity, m_highEncoderVelocity;

    private boolean m_entranceLastVal, m_transferLastVal, m_exitLastVal;

    public BallCounter(DoubleSupplier lowEncoderVelocity, DoubleSupplier highEncoderVelocity) {
        m_lowEncoderVelocity = lowEncoderVelocity;
        m_highEncoderVelocity = highEncoderVelocity;

        m_entranceIR = new DigitalInput(Constants.Magazine.ENTRANCE_IR_PORT);
        m_transferIR = new DigitalInput(Constants.Magazine.TRANSFER_IR_PORT);
        m_exitIR = new DigitalInput(Constants.Magazine.EXIT_IR_PORT);

        m_entranceCounter = new Counter(m_entranceIR);
        m_transferCounter = new Counter(m_transferIR);
        m_exitCounter = new Counter(m_exitIR);

        m_entranceCounter.setMaxPeriod(Constants.Magazine.COUNTER_MAX_PERIOD);
        m_transferCounter.setMaxPeriod(Constants.Magazine.COUNTER_MAX_PERIOD);
        m_exitCounter.setMaxPeriod(Constants.Magazine.COUNTER_MAX_PERIOD);

//        m_entranceCounter.setSamplesToAverage(Constants.Magazine.SAMPLES_TO_AVERAGE);
//        m_transferCounter.setSamplesToAverage(Constants.Magazine.SAMPLES_TO_AVERAGE);
//        m_exitCounter.setSamplesToAverage(Constants.Magazine.SAMPLES_TO_AVERAGE);

        this.addToShuffleboard();
    }

    public void addToShuffleboard() {
        ShuffleboardTab tab = Shuffleboard.getTab(Constants.Magazine.TAB_NAME);
        tab.addNumber("Low Mag Ball Count", this::getNumBallsLow);
        tab.addNumber("High Mag Ball Count", this::getNumBallsHigh);
        tab.addBoolean("Entrance IR", this::getEntranceIR);
        tab.addBoolean("Transfer IR", this::getTransferIR);
        tab.addBoolean("Exit IR", this::getExitIR);
        tab.add("Entrance Counter", m_entranceCounter);
        tab.add("Transfer Counter", m_transferCounter);
        tab.add("Exit Counter", m_exitCounter);
    }

    public int getNumBallsLow() {
        return this.m_numBallsLow;
    }

    public int getNumBallsHigh() {
        return this.m_numBallsHigh;
    }

    public void setNumBallsLow(int num) {
        this.m_numBallsLow = num;
    }

    public void setNumBallsHigh(int num) {
        this.m_numBallsHigh = num;
    }

    public boolean getEntranceIR() {
        return !m_entranceIR.get();
    }

    public boolean getTransferIR() {
        return !m_transferIR.get();
    }

    public boolean getExitIR() {
        return !m_exitIR.get();
    }

    @Override
    public void periodic() {

//        if (m_entranceCounter.get() > 0) {
        if (!m_entranceIR.get() && m_entranceLastVal) {
            if (m_lowEncoderVelocity.getAsDouble() < 0) {
//                m_numBallsLow+= m_entranceCounter.get();
                m_numBallsLow++;
            } else if (m_lowEncoderVelocity.getAsDouble() > 0) {
//                m_numBallsLow-= m_entranceCounter.get();
                m_numBallsLow--;
            }
            m_entranceCounter.reset();
        }

//        if (m_transferCounter.get() > 0) {
        if (!m_transferIR.get() && m_transferLastVal) {
            if (m_highEncoderVelocity.getAsDouble() < 0) {
//                m_numBallsHigh += m_transferCounter.get();
                m_numBallsHigh++;
            } else if (m_highEncoderVelocity.getAsDouble() > 0) {
//                m_numBallsHigh -= m_transferCounter.get();
                m_numBallsHigh--;
            }

            if (m_lowEncoderVelocity.getAsDouble() < 0) {
//                m_numBallsLow-= m_transferCounter.get();
                m_numBallsLow--;
            } else if (m_lowEncoderVelocity.getAsDouble() > 0) {
//                m_numBallsLow += m_transferCounter.get();
                m_numBallsLow++;
            }

            m_transferCounter.reset();
        }

//        if (m_exitCounter.get() > 0) {
        if (!m_exitIR.get() && m_exitLastVal) {
            if (m_highEncoderVelocity.getAsDouble() < 0) {
//                m_numBallsHigh -= m_transferCounter.get();
                m_numBallsHigh--;
            } else if (m_highEncoderVelocity.getAsDouble() > 0) {
//                m_numBallsHigh += m_transferCounter.get();
                m_numBallsHigh++;
            }
            m_exitCounter.reset();
        }

        m_entranceLastVal = m_entranceIR.get();
        m_transferLastVal = m_transferIR.get();
        m_exitLastVal = m_exitIR.get();


        if (m_numBallsHigh < 0) {
            m_numBallsHigh = 0;
        }

        if (m_numBallsLow < 0) {
            m_numBallsLow = 0;
        }
    }
}
