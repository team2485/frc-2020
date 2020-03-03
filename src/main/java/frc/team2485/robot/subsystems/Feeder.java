package frc.team2485.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.ControlType;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpiutil.math.MathUtil;
import frc.team2485.WarlordsLib.VelocityPIDSubsystem;
import frc.team2485.WarlordsLib.control.WL_PIDController;
import frc.team2485.WarlordsLib.motorcontrol.PIDSparkMax;
import frc.team2485.WarlordsLib.robotConfigs.RobotConfigs;
import frc.team2485.robot.Constants;

public class Feeder extends SubsystemBase implements VelocityPIDSubsystem {

    private PIDSparkMax m_spark;

    public Feeder() {
        this.m_spark = new PIDSparkMax(Constants.Feeder.SPARK_PORT, ControlType.kVelocity);
        this.m_spark.setSmartCurrentLimit(Constants.Feeder.MAX_CURRENT);
        this.m_spark.getEncoder().setPositionConversionFactor(Constants.Feeder.DISTANCE_PER_REVOLUTION);
        this.m_spark.getEncoder().setVelocityConversionFactor(Constants.Feeder.DISTANCE_PER_REVOLUTION / 60);

        RobotConfigs.getInstance().addConfigurable(Constants.Feeder.VELOCITY_CONTROLLER_CONFIGURABLE_LABEL, m_spark);

        this.addToShuffleboard();
    }

    public void addToShuffleboard() {
        ShuffleboardTab tab = Shuffleboard.getTab(Constants.Feeder.TAB_NAME);
        tab.add("Feeder Vel Ctrl", m_spark);
        tab.addNumber("Feeder Current", m_spark::getOutputCurrent);
        tab.addNumber("Feeder Velocity", this::getEncoderVelocity);
    }


    public void setPWM(double pwm) {
        m_spark.set(pwm);
    }

    @Override
    public void resetPIDs() {
        m_spark.resetPID();
    }

    @Override
    public void runVelocityPID(double velocity) {
        m_spark.runPID(MathUtil.clamp(velocity, Constants.Feeder.FEEDER_MIN_VELOCITY, Constants.Feeder.FEEDER_MAX_VELOCITY));
    }

    @Override
    public boolean atVelocitySetpoint() {
        return m_spark.atTarget();
    }


    public double getEncoderPosition() {
        return m_spark.getEncoder().getPosition();
    }

    @Override
    public double getEncoderVelocity() {
        return m_spark.getEncoder().getVelocity();
    }

    public CANEncoder getHoodEncoder() {
        return m_spark.getAlternateEncoder(Constants.Hood.ENCODER_CPR);
    }

    @Override
    public void tunePeriodic(int layer) {
        m_spark.runPID();
    }

}
