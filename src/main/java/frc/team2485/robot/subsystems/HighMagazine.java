package frc.team2485.robot.subsystems;

import com.revrobotics.ControlType;
import edu.wpi.first.wpilibj.Counter;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableRegistry;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team2485.WarlordsLib.Tunable;
import frc.team2485.WarlordsLib.motorcontrol.PIDSparkMax;
import frc.team2485.WarlordsLib.robotConfigs.RobotConfigs;
import frc.team2485.robot.Constants;

import java.util.function.BooleanSupplier;
import java.util.function.IntSupplier;

public class HighMagazine extends SubsystemBase implements Tunable {

    private PIDSparkMax m_spark;

    public enum MagazineState {
        INTAKING, FEEDING

    }

    private MagazineState m_state;

    /**
     * High magazine subystem, controlling the top belt stage and outtake rollers.
     *
     */
    public HighMagazine() {
        m_spark = new PIDSparkMax(Constants.Magazine.SPARK_HIGH_PORT, ControlType.kPosition);
        m_spark.getEncoder().setPositionConversionFactor(Constants.Magazine.HIGH_GEAR_RATIO * Constants.Magazine.ROLLER_RADIUS * 2 * Math.PI);
        m_spark.getEncoder().setVelocityConversionFactor(Constants.Magazine.HIGH_GEAR_RATIO * Constants.Magazine.ROLLER_RADIUS * 2 * Math.PI / 60);
        m_spark.getEncoder().setPosition(0);
        //        m_spark.setInverted(true);
        m_spark.enableVoltageCompensation(Constants.NOMINAL_VOLTAGE);

        m_state = MagazineState.INTAKING;


        RobotConfigs.getInstance().addConfigurable(Constants.Magazine.HIGH_MAGAZINE_POSITION_CONTROLLER_CONFIGURABLE_LABEL, m_spark);

        this.addToShuffleboard();
    }

    public void addToShuffleboard() {
        SendableRegistry.add(m_spark, "High Magazine Spark");

        ShuffleboardTab tab = Shuffleboard.getTab(Constants.Magazine.TAB_NAME);
        tab.add(this);
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

    public boolean setPosition(double position) {
        m_spark.runPID(position);
        return m_spark.atTarget();
    }

    public boolean atPositionSetpoint() {
        return m_spark.atTarget();
    }

    /**
     * @return belt encoder position in inches
     */
    public double getEncoderPosition() {
        return m_spark.getEncoder().getPosition();
    }

    /**
     * @return belt encoder velocity in inches per second
     */
    public double getEncoderVelocity() {
        return m_spark.getEncoder().getVelocity();
    }

    public void setMagazineState(MagazineState state) {
        this.m_state = state;
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
