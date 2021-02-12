package frc.team2485.robot.subsystems;

import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.revrobotics.CANEncoder;
import edu.wpi.first.wpilibj.Counter;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalSource;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpiutil.math.MathUtil;
import frc.team2485.WarlordsLib.PositionPIDSubsystem;
import frc.team2485.WarlordsLib.Tunable;
import frc.team2485.WarlordsLib.VelocityPIDSubsystem;
import frc.team2485.WarlordsLib.control.WL_PIDController;
import frc.team2485.WarlordsLib.motorcontrol.PIDTalonSRX;
import frc.team2485.WarlordsLib.motorcontrol.WL_TalonSRX;
import frc.team2485.WarlordsLib.motorcontrol.currentmanagement.CurrentTalonSRX;
import frc.team2485.robot.Constants;

public class IntakeArm extends SubsystemBase implements PositionPIDSubsystem, VelocityPIDSubsystem {

    private WL_TalonSRX m_talon;

    private CANEncoder m_encoder;

    private double m_lastPWM;

    private final double TOP_POSITION;

    private final double BOTTOM_POSITION;

    private WL_PIDController m_velocityController;
    private WL_PIDController m_positionController;

    public IntakeArm(CANEncoder encoder) {
        super();

        this.m_talon = new WL_TalonSRX(Constants.IntakeArm.TALON_PORT);

        this.m_talon.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyClosed);
        this.m_talon.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyClosed);

        this.m_talon.configContinuousCurrentLimit(Constants.IntakeArm.MAX_CURRENT);

        this.m_encoder = encoder;

        this.m_velocityController = new WL_PIDController();
        this.m_positionController = new WL_PIDController();

        this.TOP_POSITION = Constants.IntakeArm.TOP_POSITION_DEGREES;

        this.BOTTOM_POSITION = Constants.IntakeArm.BOTTOM_POSITION_DEGREES;

        this.addToShuffleboard();
    }

    public void addToShuffleboard() {
        ShuffleboardTab tab = Shuffleboard.getTab("Intake Arm");
        tab.add(this);
        tab.add(m_talon);
        tab.addBoolean("Top Limit Switch", this::getTopLimitSwitch);
        tab.addBoolean("Bottom Limit Switch", this::getBottomLimitSwitch);



        tab.addNumber("Output Current", m_talon::getStatorCurrent);
    }

    public void setPWM(double pwm) {
        m_talon.set(pwm);
        m_lastPWM = pwm;
    }

    @Override
    public void resetPIDs() {
    }
//
//    public double getEncoderCounts() {
//        return this.m_encoderCounts;
//    }
//
//    public double getEncoderRevolutions() {
//        return this.m_encoderCounts / this.ENCODER_COUNTS_PER_REV;
//    }
//
//    public double getEncoderDegrees() {
//        return getEncoderRevolutions() * 360;
//    }
//
//    public void setEncoderCounts(int counts) {
//        this.m_encoderCounts = counts;
//    }
//
//    public void setEncoderDegrees(double degrees) {
//        this.m_encoderCounts = (int) ((degrees / 360) * this.ENCODER_COUNTS_PER_REV);
//    }

    public boolean getTopLimitSwitch() {
        return !m_talon.getSensorCollection().isFwdLimitSwitchClosed(); //normally closed switch
    }

    public boolean getBottomLimitSwitch() {
        return !m_talon.getSensorCollection().isRevLimitSwitchClosed(); //normally closed switch
    }

    @Override
    public void runVelocityPID(double velocity) {
        this.setPWM(m_velocityController.calculate(this.getEncoderVelocity(), MathUtil.clamp(velocity, Constants.Hood.HOOD_MIN_VELOCITY, Constants.Hood.HOOD_MAX_VELOCITY)));
    }

    @Override
    public void runPositionPID(double position) {
        runVelocityPID(m_positionController.calculate(this.getEncoderPosition(), MathUtil.clamp(position, Constants.Hood.HOOD_TOP_POSITION_DEG, Constants.Hood.HOOD_BOTTOM_POSITION_DEG)));
    }

    @Override
    public boolean atVelocitySetpoint() {
        return m_velocityController.atSetpoint();
    }

    @Override
    public boolean atPositionSetpoint() {
        return m_positionController.atSetpoint();
    }

    @Override
    public double getEncoderVelocity() {
        return m_encoder.getVelocity();
    }

    @Override
    public double getEncoderPosition() {
        return m_encoder.getPosition();
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
//    private void updateEncoderCounts() {
//        if (m_lastPWM > 0) {
//            m_encoderCounts += m_encoderCounter.get();
//        } else if (m_lastPWM < 0) {
//            m_encoderCounts -= m_encoderCounter.get();
//        }
//        m_encoderCounter.reset();
//    }

    @Override
    public void periodic() {
//        if (getTopLimitSwitch()) {
//            setEncoderDegrees(TOP_POSITION);
//        } else if (getBottomLimitSwitch()) {
//            setEncoderDegrees(BOTTOM_POSITION);
//        } else {
//            updateEncoderCounts();
//        }
    }

    /**
     * Should run periodically and run the motor to tune
     */
    @Override
    public void tunePeriodic(int layer) {
        if (layer == 0) {
            m_talon.set(m_velocityController.calculate(this.getEncoderVelocity()));
        } else if (layer == 1) {
            runVelocityPID(m_positionController.calculate(this.getEncoderPosition()));
        }
    }
}
