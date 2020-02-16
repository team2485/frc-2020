package frc.team2485.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableRegistry;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team2485.WarlordsLib.Tunable;
import frc.team2485.WarlordsLib.control.WL_PIDController;
import frc.team2485.WarlordsLib.motorcontrol.PIDTalonSRX;
import frc.team2485.WarlordsLib.motorcontrol.currentmanagement.CurrentTalonSRX;
import frc.team2485.WarlordsLib.oi.Deadband;
import frc.team2485.WarlordsLib.robotConfigs.RobotConfigs;
import frc.team2485.robot.Constants;

import java.util.function.BooleanSupplier;

public class HighMagazine extends SubsystemBase implements AbstractMagazinePart, Tunable {

    private CurrentTalonSRX m_talon;

    private WL_PIDController m_positionController;

    private DigitalInput m_exitIR;

    /**
     * the number of balls currently contained in the high belt
     */
    private int m_numBalls;

    private BooleanSupplier m_transferIR;

    private boolean m_transferIRLastVal, m_exitIRLastVal;

    /**
     * High magazine subystem, controlling the top belt stage and outtake rollers.
     * @param transferIR boolean supplier for the beam break sensor at the intersection of
     *                   the low and high belts, provided through the LowMagazine subsystem.
     */
    public HighMagazine(BooleanSupplier transferIR)  {
        m_talon = new CurrentTalonSRX(Constants.Magazine.TALON_HIGH_PORT, Constants.Magazine.TALON_HIGH_MAX_CURRENT);

        m_exitIR = new DigitalInput(Constants.Magazine.EXIT_IR_PORT);

        m_positionController = new WL_PIDController();

        m_numBalls = 0;

        m_transferIR = transferIR;

        SendableRegistry.add(m_positionController, "High Magazine Position Controller");
        RobotConfigs.getInstance().addConfigurable("highMagazinePositionController", m_positionController);


        ShuffleboardTab tab = Shuffleboard.getTab("Magazine");
        tab.addNumber("High Position", this::getEncoderPosition);
        tab.addNumber("High Velocity", this::getEncoderPosition);
        tab.addNumber("High Number of Balls", this::getNumBalls);
        tab.addBoolean("Exit IR", this::getTransferIR);
    }

    /**
     * Sets talon to a specific PWM
     * @param pwm PWM to set the talon to
     */
    public void setPWM(double pwm) {
        m_talon.setPWM(pwm);
    }

    /**
     * Sets the talon to a specific current using PID control
     * @param current current to set the PID to
     */
    public void setCurrent(double current) {
        m_talon.setCurrent(current);
    }

    @Override
    public boolean setPosition(double position) {
        setPWM(m_positionController.calculate(getEncoderPosition(), position));
        return m_positionController.atSetpoint();
    }

    @Override
    public boolean atPositionSetpoint() {
        return m_positionController.atSetpoint();
    }

    @Override
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
     * @return value of beam break sensor at intersection of low and high belts
     */
    public boolean getTransferIR() {
        return m_transferIR.getAsBoolean();
    }

    /**
     *
     * @return boolean value of beam break sensor at end of high belt
     */
    public boolean getExitIR() {
        return m_exitIR.get();
    }

    /**
     *
     * @return boolean number of balls currently in the high belt
     */
    public int getNumBalls() {
        return m_numBalls;
    }

    /**
     * Periodic method updating the number of balls in the high belt using beam break sensors.
     */
    @Override
    public void periodic() {

        if (!getTransferIR() && getTransferIR() != m_transferIRLastVal) {
            if (getEncoderVelocity() < 0) {
                m_numBalls++;
            } else if (getEncoderVelocity() > 0) {
                m_numBalls--;
            }
        }

        if (!getExitIR() && getExitIR() != m_exitIRLastVal) {
            if (getEncoderVelocity() < 0) {
                m_numBalls--;
            } else if (getEncoderVelocity() > 0 ) {
                m_numBalls++;
            }
        }

        m_exitIRLastVal = getExitIR();
        m_transferIRLastVal = getTransferIR();
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
//            m_positionController.calculate(getEncoderPosition());
        } else {
            this.setPWM(0);
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
