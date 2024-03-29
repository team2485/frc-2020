package frc.team2485.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpiutil.math.MathUtil;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.team2485.WarlordsLib.Tunable;
import frc.team2485.WarlordsLib.motorcontrol.PIDSparkMax;
import frc.team2485.WarlordsLib.robotConfigs.Configurable;
import frc.team2485.WarlordsLib.robotConfigs.LoadableConfigs;
import frc.team2485.WarlordsLib.robotConfigs.RobotConfigs;
import frc.team2485.WarlordsLib.robotConfigs.SavableConfigs;
import frc.team2485.WarlordsLib.Debounce;
import frc.team2485.robot.Constants;

//Flywheels handles IR ball sensing

public class Flywheels extends SubsystemBase implements Tunable, Configurable {

    private PIDSparkMax m_sparkLeft;
    private PIDSparkMax m_sparkRight;

    /*
    * INDEXING HANDLED THROUGH FLYWHEELS
    * Entrance IR handles input, increments ball count 
    * Current dips while running flywheels at speed indicate ball output
    */ 
    private int m_numBalls; 
    private boolean m_ballsAtTop;
    private DigitalInput m_entranceIR, m_transferIR, m_exitIR;
    private Debounce m_entranceDebounce, m_transferDebounce, m_exitDebounce;
    private boolean m_entranceVal, m_transferVal, m_exitVal, m_entranceLastVal;
    private boolean m_isShooting;

    private double m_setpoint;

    public Flywheels() {
        this.m_sparkLeft = new PIDSparkMax(Constants.Flywheels.SPARK_FLYWHEEL_LEFT_PORT, ControlType.kVelocity);
        this.m_sparkRight = new PIDSparkMax(Constants.Flywheels.SPARK_FLYWHEEL_RIGHT_PORT, ControlType.kVelocity);
        this.m_sparkRight = new PIDSparkMax(Constants.Flywheels.SPARK_FLYWHEEL_RIGHT_PORT, ControlType.kVelocity);
        m_sparkRight.setInverted(true);

        m_sparkRight.getEncoder().setVelocityConversionFactor(Constants.Flywheels.GEAR_RATIO);
        m_sparkLeft.getEncoder().setVelocityConversionFactor(Constants.Flywheels.GEAR_RATIO);

        m_sparkLeft.setIdleMode(CANSparkMax.IdleMode.kCoast);
        m_sparkRight.setIdleMode(CANSparkMax.IdleMode.kCoast);

        m_sparkRight.setSmartCurrentLimit(Constants.Flywheels.SPARK_FLYWHEEL_LEFT_MAX_CURRENT);
        m_sparkLeft.setSmartCurrentLimit(Constants.Flywheels.SPARK_FLYWHEEL_RIGHT_MAX_CURRENT);

        m_sparkLeft.enableVoltageCompensation(Constants.NOMINAL_VOLTAGE);
        m_sparkRight.enableVoltageCompensation(Constants.NOMINAL_VOLTAGE);

        m_sparkRight.setTolerance(Constants.Flywheels.RPM_THRESHOLD);
        m_sparkLeft.setTolerance(Constants.Flywheels.RPM_THRESHOLD);

        RobotConfigs.getInstance().addConfigurable(Constants.Flywheels.LEFT_VELOCITY_CONTROLLER_CONFIGURABLE_LABEL, m_sparkLeft);
        RobotConfigs.getInstance().addConfigurable(Constants.Flywheels.RIGHT_VELOCITY_CONTROLLER_CONFIGURABLE_LABEL, m_sparkRight);


        //BALL HANDLING
        m_entranceIR = new DigitalInput(Constants.Flywheels.ENTRANCE_IR_PORT);
        m_transferIR = new DigitalInput(Constants.Flywheels.TRANSFER_IR_PORT);
        m_exitIR = new DigitalInput(Constants.Flywheels.EXIT_IR_PORT);
        m_entranceDebounce = new Debounce(m_entranceIR.get(), Constants.Flywheels.MAX_DEBOUNCE_TIME);
        m_transferDebounce = new Debounce(m_transferIR.get(), Constants.Flywheels.MAX_DEBOUNCE_TIME);
        m_isShooting = false;
        m_numBalls = 0;
        m_ballsAtTop = false;

        m_setpoint = 0;

        this.addToShuffleboard();
    }

    private void addToShuffleboard() {
        ShuffleboardTab flywheels = Shuffleboard.getTab(Constants.Flywheels.FLYWHEELS_TAB_NAME);
        flywheels.add("Left Flywheel Vel Ctrl", m_sparkLeft);
        flywheels.add("Right Flywheel Vel Ctrl", m_sparkRight);
        flywheels.addNumber("Left Flywheel Velocity", this::getLeftEncoderVelocity);
        flywheels.addNumber("Right Flywheel Velocity", this::getRightEncoderVelocity);
        flywheels.addNumber("Left Flywheel Current", m_sparkLeft::getOutputCurrent);
        flywheels.addNumber("Right Flywheel Current", m_sparkRight::getOutputCurrent);
        flywheels.addBoolean("At target?", this::atVelocitySetpoint);
        
        ShuffleboardTab indexing = Shuffleboard.getTab(Constants.Flywheels.INDEXING_TAB_NAME);
        indexing.addBoolean("Entrance IR Has Ball?", this::entranceIRHasBall);
        indexing.addBoolean("Transfer IR Has Ball?", this::transferIRHasBall);
        indexing.addBoolean("Exit IR Has Ball?", this::exitIRHasBall);
        indexing.addBoolean("Entrance IR Value", m_entranceIR::get);
        indexing.addBoolean("Transfer IR Value", m_transferIR::get);
        indexing.addBoolean("Exit IR Value", m_exitIR::get);



        //indexing.add("Arm and Trigger (Flywheel Velocity)", m_exitAT);
        //tab.addNumber("Right Flywheel Error", this.getRightEncoderVelocity() - Constants.Setpoints.FARRPM);
    }

    private void setLeftPWM(double pwm) {
        m_sparkLeft.set(pwm);
    }

    private void setRightPWM(double pwm) {
        m_sparkRight.set(pwm);
    }

    public void setPWM(double leftPWM, double rightPWM) {
        setLeftPWM(leftPWM);
        setRightPWM(rightPWM);
    }

    public void setPWM(double pwm) {
        setPWM(pwm, pwm);
    }

    @Override
    public void resetPIDs() {
        m_sparkLeft.resetPID();
        m_sparkRight.resetPID();
    }

    private void setLeftVelocity(double velocity) {
        m_sparkLeft.runPID(MathUtil.clamp(velocity, Constants.Flywheels.FLYWHEELS_MIN_VELOCITY, Constants.Flywheels.FLYWHEELS_MAX_VELOCITY));
    }

    private void setRightVelocity(double velocity) {
        m_sparkRight.runPID(MathUtil.clamp(velocity, Constants.Flywheels.FLYWHEELS_MIN_VELOCITY, Constants.Flywheels.FLYWHEELS_MAX_VELOCITY));
    }

    public void setVelocity(double velocity) {

        setVelocity(velocity, velocity);
    }

    public void setVelocity(double leftVelocity, double rightVelocity) {
        setLeftVelocity(leftVelocity);
        setRightVelocity(rightVelocity);
    }

    public boolean atVelocitySetpoint() {
        return m_sparkLeft.atTarget() && m_sparkRight.atTarget();
    }

    public double getLeftEncoderVelocity() {
        return m_sparkLeft.getEncoder().getVelocity();
    }

    public double getRightEncoderVelocity() {
        return m_sparkRight.getEncoder().getVelocity();
    }

    /**
     * True if ball is in IR
     */
    public boolean entranceIRHasBall() {
        return m_entranceVal;
    }

    /**
     * True if ball is in IR
     */
    public boolean transferIRHasBall() {
        return m_transferVal;
    }

    public boolean exitIRHasBall() {
        return m_exitVal;
    }

    public void incrementBalls(boolean pos) {
        m_numBalls = pos ? m_numBalls + 1 : m_numBalls - 1;
    }

    public void updateBallPosition(boolean ballsAtTop) {
        m_ballsAtTop = ballsAtTop;
    }

    public boolean getBallPosition() {
        return m_ballsAtTop;
    }

    public void setSetpoint(double setpoint) {
        m_setpoint = setpoint;
    }

    public double getSetpoint() {
        return m_setpoint;
    }
    @Override
    public void periodic() {
        m_transferVal = !m_transferIR.get();
        m_exitVal = !m_exitIR.get();
    }

    @Override
    public void tunePeriodic(int layer) {
        m_sparkLeft.runPID();
        m_sparkRight.runPID();
    }

    @Override
    public void loadConfigs(LoadableConfigs configs) {

    }

    @Override
    public void saveConfigs(SavableConfigs configs) {

    }
}
