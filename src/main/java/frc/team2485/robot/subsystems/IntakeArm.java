package frc.team2485.robot.subsystems;

import edu.wpi.first.wpilibj.Counter;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team2485.WarlordsLib.Tunable;
import frc.team2485.WarlordsLib.motorcontrol.WL_TalonSRX;
import frc.team2485.WarlordsLib.motorcontrol.currentmanagement.CurrentTalonSRX;
import frc.team2485.robot.Constants;

public class IntakeArm extends SubsystemBase implements Tunable {

    private CurrentTalonSRX m_talon;

    /**
     * FPGA-run counter to count encoder pulses.
     */
    private Counter m_encoderCounter;

    private int m_encoderCounts;

    private double m_lastPWM;

    private final double ENCODER_COUNTS_PER_REV;

    private final double TOP_POSITION;

    private final double BOTTOM_POSITION;

    public IntakeArm() {
        super();

        this.m_talon = new CurrentTalonSRX(Constants.IntakeArm.SPARK_PORT, Constants.IntakeArm.MAX_CURRENT);

        this.m_encoderCounter = new Counter(Constants.IntakeArm.ENCODER_DIO_PORT);

        this.ENCODER_COUNTS_PER_REV = Constants.IntakeArm.ENCODER_PULSES_PER_REVOLUTION * 2; // counter does 2x encoding.

        this.TOP_POSITION = Constants.IntakeArm.TOP_POSITION_DEGREES;

        this.BOTTOM_POSITION = Constants.IntakeArm.BOTTOM_POSITION_DEGREES;
    }

    public void setPWM(double pwm) {
        m_talon.setPWM(pwm);
        m_lastPWM = pwm;
    }

    public double getEncoderCounts() {
        return this.m_encoderCounts;
    }

    public double getEncoderRevolutions() {
        return this.m_encoderCounts / this.ENCODER_COUNTS_PER_REV;
    }

    public double getEncoderDegrees() {
        return getEncoderRevolutions() * 360;
    }

    public void setEncoderCounts(int counts) {
        this.m_encoderCounts = counts;
    }

    public void setEncoderDegrees(double degrees) {
        this.m_encoderCounts = (int)((degrees / 360) * this.ENCODER_COUNTS_PER_REV);
    }

    public boolean getTopLimitSwitch() {
        return m_talon.getSensorCollection().isRevLimitSwitchClosed();
    }

    public boolean getBottomLimitSwitch() {
        return m_talon.getSensorCollection().isFwdLimitSwitchClosed();
    }

    public double getTopPosition() {
        return this.TOP_POSITION;
    }

    public double getBottomPosition() {
        return this.BOTTOM_POSITION;
    }

    /**
     * Update encoder counts using counter and the last set position. Works best when run periodically whenever you set the motor!
     */
    private void updateEncoderCounts() {
        if (m_lastPWM > 0) {
            m_encoderCounts += m_encoderCounter.get();
        } else if (m_lastPWM < 0) {
            m_encoderCounts -= m_encoderCounter.get();
        }
        m_encoderCounter.reset();
    }

    @Override
    public void periodic() {
        if (getTopLimitSwitch()) {
           setEncoderDegrees(TOP_POSITION);
        } else if (getBottomLimitSwitch()) {
            setEncoderDegrees(BOTTOM_POSITION);
        } else {
            updateEncoderCounts();
        }
    }

    /**
     * Should run periodically and run the motor to tune when enabled
     *
     * @param enable use this parameter to only enable the motor when this is true.
     */
    @Override
    public void tunePeriodic(boolean enable) {
        if (enable) {
            m_talon.runPID();
        } else {
            m_talon.set(0);
        }
    }

    /**
     * Set the raw PWM of the subsystem motor. This is for setting the PWM WITHOUT using any current-based pwm.
     *
     * @param pwm value between -1 and 1
     */
    @Override
    public void setRawPWM(double pwm) {
        m_talon.set(pwm);
    }
}
