package frc.team2485.robot.subsystems;

import com.revrobotics.ControlType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableRegistry;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team2485.WarlordsLib.Tunable;
import frc.team2485.WarlordsLib.motorcontrol.PIDSparkMax;
import frc.team2485.WarlordsLib.robotConfigs.RobotConfigs;
import frc.team2485.robot.Constants;

public class LowMagazine extends SubsystemBase implements Tunable {

    private PIDSparkMax m_spark;

    private DigitalInput m_entranceIR, m_transferIR;

    /**
     * the number of balls currently contained in the low belt
     */
    private int m_numBalls;

    private boolean m_entranceIRLastVal, m_transferIRLastVal;

    /**
     * Low magazine subystem, controlling the intake rollers and low belt.
     */
    public LowMagazine() {
        m_spark = new PIDSparkMax(Constants.Magazine.SPARK_LOW_PORT, ControlType.kCurrent);
        m_spark.getEncoder().setPositionConversionFactor(Constants.Magazine.LOW_GEAR_RATIO * Constants.Magazine.ROLLER_RADIUS * 2 * Math.PI);
        m_spark.getEncoder().setVelocityConversionFactor(Constants.Magazine.LOW_GEAR_RATIO * Constants.Magazine.ROLLER_RADIUS * 2 * Math.PI / 60);
        m_spark.setInverted(true);
        m_spark.enableVoltageCompensation(Constants.NOMINAL_VOLTAGE);
        m_spark.setEncoderPosition(0);

        m_entranceIR = new DigitalInput(Constants.Magazine.ENTRANCE_IR_PORT);
        m_transferIR = new DigitalInput(Constants.Magazine.TRANSFER_IR_PORT);

        m_numBalls = 0;

        m_entranceIRLastVal = false;

        RobotConfigs.getInstance().addConfigurable(Constants.Magazine.LOW_MAGAZINE_VELOCITY_CONTROLLER_CONFIGURABLE_LABEL, m_spark);

        this.addToShuffleboard();
    }

    public void addToShuffleboard() {
        SendableRegistry.add(m_spark, "Low Magazine Spark");
        ShuffleboardTab tab = Shuffleboard.getTab(Constants.Magazine.TAB_NAME);
        tab.add(m_spark);
        tab.addNumber("Low Position", this::getEncoderPosition);
        tab.addNumber("Low Velocity", this::getEncoderVelocity);
        tab.addNumber("Low Current", m_spark::getOutputCurrent);
        tab.addNumber("Low Number of Balls", this::getNumBalls);
        tab.addBoolean("Entrance IR", this::getEntranceIR);
        tab.addBoolean("Transfer IR", this::getTransferIR);
    }

    /**
     * Sets talon to a specific PWM
     * @param pwm PWM to set the talon to
     */
    public void setPWM(double pwm) {
        m_spark.set(pwm);
    }


    public boolean setVelocity(double velocity) {
        m_spark.runPID(velocity);
        return m_spark.atTarget();
    }

    /**
     * @return belt encoder position
     */
    public double getEncoderPosition() {
        return m_spark.getEncoder().getPosition();
    }


    public boolean atVelocitySetpoint() {
        return m_spark.atTarget();
    }

    /**
     *
     * @return belt encoder velocity in inches per second
     */
    public double getEncoderVelocity() {
        return m_spark.getEncoder().getVelocity();
    }

    /**
     *
     * @return boolean value of beam break sensor at start of low belt
     */
    public boolean getEntranceIR() {
        return !m_entranceIR.get();
    }


    /**
     *
     * @return value of beam break sensor at intersection of low and high belts
     */
    public boolean getTransferIR() {
        return !m_transferIR.get();
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

    /**
     * Should run periodically and run the motor to tune when enabled
     */
    @Override
    public void tunePeriodic() {
        m_spark.runPID();

    }
}
