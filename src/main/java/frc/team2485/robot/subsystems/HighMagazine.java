package frc.team2485.robot.subsystems;

import com.revrobotics.ControlType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableRegistry;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team2485.WarlordsLib.Tunable;
import frc.team2485.WarlordsLib.motorcontrol.PIDSparkMax;
import frc.team2485.WarlordsLib.motorcontrol.PIDTalonSRX;
import frc.team2485.WarlordsLib.oi.Deadband;
import frc.team2485.WarlordsLib.robotConfigs.RobotConfigs;
import frc.team2485.robot.Constants;

import java.util.function.BooleanSupplier;

public class HighMagazine extends SubsystemBase implements Tunable {

    private PIDSparkMax m_spark;

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
        m_spark = new PIDSparkMax(Constants.Magazine.SPARK_HIGH_PORT, ControlType.kPosition);
        m_spark.getEncoder().setPositionConversionFactor(Constants.Magazine.HIGH_GEAR_RATIO * Constants.Magazine.ROLLER_DIAMETER * 2 * Math.PI);
        m_spark.getEncoder().setVelocityConversionFactor(Constants.Magazine.HIGH_GEAR_RATIO * Constants.Magazine.ROLLER_DIAMETER * 2 * Math.PI);
        m_spark.setInverted(true);
        m_spark.enableVoltageCompensation(Constants.NOMINAL_VOLTAGE);

        m_exitIR = new DigitalInput(Constants.Magazine.EXIT_IR_PORT);

        m_numBalls = 0;

        m_transferIR = transferIR;

        RobotConfigs.getInstance().addConfigurable(Constants.Magazine.HIGH_MAGAZINE_POSITION_CONTROLLER_CONFIGURABLE_LABEL, m_spark);

        this.addToShuffleboard();
    }

    public void addToShuffleboard() {
        SendableRegistry.add(m_spark, "High Magazine Spark");

        ShuffleboardTab tab = Shuffleboard.getTab(Constants.Magazine.TAB_NAME);
        tab.add(this);
        tab.addNumber("High Position", this::getEncoderPosition);
        tab.addNumber("High Velocity", this::getEncoderVelocity);
        tab.addNumber("High Current", m_spark::getOutputCurrent);
        tab.addNumber("High Number of Balls", this::getNumBalls);
        tab.addBoolean("Exit IR", this::getExitIR);
    }

    /**
     * Sets talon to a specific PWM
     * @param pwm PWM to set the talon to
     */
    public void setPWM(double pwm) {
        m_spark.set(pwm);
    }

    public boolean setPosition(double position) {
        m_spark.runPID(position);
        return m_spark.atTarget();
    }

    public boolean atPositionSetpoint() {
        return m_spark.atTarget();
    }

    /**
     *
     * @return belt encoder position in inches
     */
    public double getEncoderPosition() {
        return m_spark.getEncoder().getPosition();
    }

    /**
     *
     * @return belt encoder velocity in inches per second
     */
    public double getEncoderVelocity() {
        return m_spark.getEncoder().getVelocity() / 60;
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
        return !m_exitIR.get();
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

//        if (!getExitIR() && getExitIR() != m_exitIRLastVal) {
//            if (getEncoderVelocity() < 0) {
//                m_numBalls--;
//            } else if (getEncoderVelocity() > 0 ) {
//                m_numBalls++;
//            }
//        }

        m_exitIRLastVal = getExitIR();
        m_transferIRLastVal = getTransferIR();
    }

    public void resetEncoder(double position) {
        m_spark.getEncoder().setPosition(0);
    }

    /**
     * Should run periodically and run the motor to tune when enabled
     */
    @Override
    public void tunePeriodic() {
        m_spark.runPID();
    }
}
