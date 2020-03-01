package frc.team2485.robot.subsystems;

import com.revrobotics.ControlType;
import edu.wpi.first.wpilibj.Counter;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableRegistry;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpiutil.math.MathUtil;
import frc.team2485.WarlordsLib.Tunable;
import frc.team2485.WarlordsLib.VelocityPIDSubsystem;
import frc.team2485.WarlordsLib.motorcontrol.PIDSparkMax;
import frc.team2485.WarlordsLib.robotConfigs.RobotConfigs;
import frc.team2485.robot.Constants;

public class LowMagazine extends SubsystemBase implements VelocityPIDSubsystem {

    private PIDSparkMax m_spark;

    /**
     * Low magazine subystem, controlling the intake rollers and low belt.
     */
    public LowMagazine() {
        m_spark = new PIDSparkMax(Constants.Magazine.SPARK_LOW_PORT, ControlType.kCurrent);
        m_spark.getEncoder().setPositionConversionFactor(Constants.Magazine.LOW_GEAR_RATIO * Constants.Magazine.ROLLER_RADIUS * 2 * Math.PI);
        m_spark.getEncoder().setVelocityConversionFactor(Constants.Magazine.LOW_GEAR_RATIO * Constants.Magazine.ROLLER_RADIUS * 2 * Math.PI / 60);
        m_spark.getEncoder().setPosition(0);
        m_spark.setInverted(true);
        m_spark.enableVoltageCompensation(Constants.NOMINAL_VOLTAGE);
        m_spark.setEncoderPosition(0);
        m_spark.setSmartCurrentLimit(Constants.Magazine.SPARK_LOW_MAX_CURRENT);

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
    }

    /**
     * Sets talon to a specific PWM
     * @param pwm PWM to set the talon to
     */
    public void setPWM(double pwm) {
        m_spark.set(pwm);
    }

    /**
     * @return belt encoder position
     */
    public double getEncoderPosition() {
        return m_spark.getEncoder().getPosition();
    }

    @Override
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

    @Override
    public void runVelocityPID(double velocity) {
        m_spark.runPID(MathUtil.clamp(velocity, Constants.Magazine.MAGAZINE_MIN_VELOCITY, Constants.Magazine.MAGAZINE_MAX_VELOCITY));
    }


    /**
     * Should run periodically and run the motor to tune when enabled
     */
    @Override
    public void tunePeriodic(int layer) {
        m_spark.runPID();

    }
}
