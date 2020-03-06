package frc.team2485.robot.subsystems;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableRegistry;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpiutil.math.MathUtil;
import frc.team2485.WarlordsLib.PositionPIDSubsystem;
import frc.team2485.WarlordsLib.VelocityPIDSubsystem;
import frc.team2485.WarlordsLib.control.WL_PIDController;
import frc.team2485.WarlordsLib.motorcontrol.WL_SparkMax;
import frc.team2485.WarlordsLib.robotConfigs.RobotConfigs;
import frc.team2485.robot.Constants;

public class HighMagazine extends SubsystemBase implements PositionPIDSubsystem, VelocityPIDSubsystem {

    private WL_SparkMax m_spark;

    private WL_PIDController m_velocityController;

    private WL_PIDController m_positionController;

    /**
     * High magazine subystem, controlling the top belt stage and outtake rollers.
     *
     */
    public HighMagazine() {
        m_spark = new WL_SparkMax(Constants.Magazine.SPARK_HIGH_PORT);

        m_spark.getEncoder().setPositionConversionFactor(Constants.Magazine.HIGH_DISTANCE_PER_REVOLUTION);
        m_spark.getEncoder().setVelocityConversionFactor(Constants.Magazine.HIGH_DISTANCE_PER_REVOLUTION / 60);
        m_spark.getEncoder().setPosition(0);
        m_spark.enableVoltageCompensation(Constants.NOMINAL_VOLTAGE);
        m_spark.setSmartCurrentLimit(Constants.Magazine.SPARK_HIGH_MAX_CURRENT);
        //m_spark.setIzone(0);


        m_velocityController = new WL_PIDController();

        m_positionController = new WL_PIDController();
        m_positionController.setTolerance(Constants.Magazine.HIGH_MAGAZINE_POSITION_THRESHOLD);

        RobotConfigs.getInstance().addConfigurable(Constants.Magazine.HIGH_MAGAZINE_VELOCITY_CONTROLLER_CONFIGURABLE_LABEL, m_velocityController);
        RobotConfigs.getInstance().addConfigurable(Constants.Magazine.HIGH_MAGAZINE_POSITION_CONTROLLER_CONFIGURABLE_LABEL, m_positionController);

        this.addToShuffleboard();
    }

    public void addToShuffleboard() {
        SendableRegistry.add(m_spark, "High Magazine Spark");

        ShuffleboardTab tab = Shuffleboard.getTab(Constants.Magazine.TAB_NAME);
        tab.add(this);
        tab.add("High Velocity Controller", m_velocityController);
        tab.add("High Position Controller", m_positionController);
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

    @Override
    public void resetPIDs() {
        m_velocityController.reset();
        m_positionController.reset();
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
        this.runVelocityPID(m_positionController.calculate(getEncoderPosition(), position));
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
//        m_spark.runPID(MathUtil.clamp(velocity, Constants.Magazine.MAGAZINE_MIN_VELOCITY, Constants.Magazine.MAGAZINE_MAX_VELOCITY));
        m_spark.set(m_velocityController.calculate(getEncoderVelocity(), MathUtil.clamp(velocity, Constants.Magazine.MAGAZINE_MIN_VELOCITY, Constants.Magazine.MAGAZINE_MAX_VELOCITY)));
    }

    @Override
    public boolean atVelocitySetpoint() {
        return m_velocityController.atSetpoint();
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

            m_spark.set(m_velocityController.calculate(getEncoderVelocity()));
        } else if (layer == 1) {

            runVelocityPID(m_positionController.calculate(getEncoderPosition()));
        }
    }
}
