package frc.team2485.robot.subsystems;

import com.revrobotics.ControlType;
import edu.wpi.first.wpilibj.Counter;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableRegistry;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team2485.WarlordsLib.PositionPIDSubsystem;
import frc.team2485.WarlordsLib.Tunable;
import frc.team2485.WarlordsLib.VelocityPIDSubsystem;
import frc.team2485.WarlordsLib.control.WL_PIDController;
import frc.team2485.WarlordsLib.motorcontrol.PIDSparkMax;
import frc.team2485.WarlordsLib.robotConfigs.RobotConfigs;
import frc.team2485.robot.Constants;

import java.util.function.BooleanSupplier;
import java.util.function.IntSupplier;

public class HighMagazine extends SubsystemBase implements PositionPIDSubsystem, VelocityPIDSubsystem {

    private PIDSparkMax m_spark;

    private WL_PIDController m_positionController;

    public enum MagazineState {
        INTAKING, FEEDING
    }

    private MagazineState m_state;

    /**
     * High magazine subystem, controlling the top belt stage and outtake rollers.
     *
     */
    public HighMagazine() {
        m_spark = new PIDSparkMax(Constants.Magazine.SPARK_HIGH_PORT, ControlType.kVelocity);

        m_spark.getEncoder().setPositionConversionFactor(Constants.Magazine.HIGH_DISTANCE_PER_REVOLUTION);
        m_spark.getEncoder().setVelocityConversionFactor(Constants.Magazine.HIGH_DISTANCE_PER_REVOLUTION / 60);
        m_spark.getEncoder().setPosition(0);
        //        m_spark.setInverted(true);
        m_spark.enableVoltageCompensation(Constants.NOMINAL_VOLTAGE);
        m_spark.setEncoderPosition(0);
        m_spark.setTolerance(Constants.Magazine.HIGH_MAGAZINE_POSITION_CONTROLLER_THRESHOLD);
        m_spark.setSmartCurrentLimit(Constants.Magazine.SPARK_HIGH_MAX_CURRENT);

        m_state = MagazineState.INTAKING;

        m_positionController = new WL_PIDController();
        m_positionController.setTolerance(0.5);

        RobotConfigs.getInstance().addConfigurable(Constants.Magazine.HIGH_MAGAZINE_VELOCITY_CONTROLLER_CONFIGURABLE_LABEL, m_spark);
        RobotConfigs.getInstance().addConfigurable(Constants.Magazine.HIGH_MAGAZINE_POSITION_CONTROLLER_CONFIGURABLE_LABEL, m_positionController);


        this.addToShuffleboard();
    }

    public void addToShuffleboard() {
        SendableRegistry.add(m_spark, "High Magazine Spark");

        ShuffleboardTab tab = Shuffleboard.getTab(Constants.Magazine.TAB_NAME);
        tab.add(this);
        tab.add("High Velocity Controller", m_spark);
        tab.add("High Position Controller", m_positionController);
        tab.addString("Magazine State", m_state::toString);
        tab.addNumber("High Position", this::getEncoderPosition);
        tab.addNumber("High Velocity", this::getEncoderVelocity);
        tab.addNumber("High Current", m_spark::getOutputCurrent);
    }

    /**
     * Sets talon to a specific PWM
     *
     * @param pwm PWM to set the talon to
     */
    public void setPWM(double pwm) {
        m_spark.set(pwm);
    }

    public boolean atPositionSetpoint() {
        return m_positionController.atSetpoint();
    }

    /**
     * @return belt encoder position in inches
     */
    public double getEncoderPosition() {
        return m_spark.getEncoder().getPosition();
    }

    @Override
    public void runPositionPID(double position) {
        runVelocityPID(m_positionController.calculate(getEncoderPosition(), position));
    }

    /**
     * @return belt encoder velocity in inches per second
     */
    @Override
    public double getEncoderVelocity() {
        return m_spark.getEncoder().getVelocity();
    }

    @Override
    public void runVelocityPID(double velocity) {
        m_spark.runPID(velocity);
    }

    @Override
    public boolean atVelocitySetpoint() {
        return m_spark.atTarget();
    }

    public void resetEncoder(double position) {
        m_spark.getEncoder().setPosition(0);
    }

    /**
     * Should run periodically and run the motor to tune when enabled
     */
    @Override
    public void tunePeriodic(int layer) {
        if (layer == 0) {
            m_spark.runPID();
        } else if (layer == 1) {
            runVelocityPID(m_positionController.calculate(getEncoderPosition()));
        }
    }
}
