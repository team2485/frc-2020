package frc.team2485.robot.subsystems;

import com.ctre.phoenix.sensors.PigeonIMU;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableRegistry;
import frc.team2485.WarlordsLib.Tunable;
import frc.team2485.WarlordsLib.motorcontrol.WL_SparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team2485.WarlordsLib.sensors.SparkMaxAlternateEncoder;
import frc.team2485.robot.Constants;


public class Drivetrain extends SubsystemBase implements Tunable {

    private DifferentialDrive m_drive;

    private WL_SparkMax m_sparkLeft1Master;
    private WL_SparkMax m_sparkLeft2;
    private WL_SparkMax m_sparkLeft3;

    private WL_SparkMax m_sparkRight1Master;
    private WL_SparkMax m_sparkRight2;
    private WL_SparkMax m_sparkRight3;

    private SparkMaxAlternateEncoder m_encoderLeft;
    private SparkMaxAlternateEncoder m_encoderRight;

    private PigeonIMU m_pigeon;

    public Drivetrain() {
        this.m_sparkLeft1Master = new WL_SparkMax(Constants.Drivetrain.SPARK_LEFT_PORT_MASTER);
        this.m_sparkLeft2 = new WL_SparkMax(Constants.Drivetrain.SPARK_LEFT_PORT_SLAVE_2);
        this.m_sparkLeft3 = new WL_SparkMax(Constants.Drivetrain.SPARK_LEFT_PORT_SLAVE_3);

        this.m_sparkRight1Master = new WL_SparkMax(Constants.Drivetrain.SPARK_RIGHT_PORT_MASTER);
        this.m_sparkRight2 = new WL_SparkMax(Constants.Drivetrain.SPARK_RIGHT_PORT_SLAVE_2);
        this.m_sparkRight3 = new WL_SparkMax(Constants.Drivetrain.SPARK_RIGHT_PORT_SLAVE_3);

        this.m_sparkLeft1Master.setSmartCurrentLimit(Constants.Drivetrain.MAX_CURRENT);
        this.m_sparkRight1Master.setSmartCurrentLimit(Constants.Drivetrain.MAX_CURRENT);

        this.m_sparkLeft1Master.setFollowers(m_sparkLeft2, m_sparkLeft3);
        this.m_sparkRight1Master.setFollowers(m_sparkRight2, m_sparkRight3);

        this.m_drive = new DifferentialDrive(m_sparkLeft1Master, m_sparkRight1Master);

        // 4x encoding so * 4
        this.m_encoderLeft = new SparkMaxAlternateEncoder(Constants.Drivetrain.LEFT_ENCODER_SPARK, Constants.Drivetrain.ENCODER_CPR);
        this.m_encoderRight = new SparkMaxAlternateEncoder(Constants.Drivetrain.RIGHT_ENCODER_SPARK, Constants.Drivetrain.ENCODER_CPR);

        this.m_encoderRight.setInverted(true);

        this.m_encoderLeft.setDistancePerRevolution(2 * Math.PI * Constants.Drivetrain.WHEEL_RADIUS);
        this.m_encoderRight.setDistancePerRevolution(2 * Math.PI * Constants.Drivetrain.WHEEL_RADIUS);

        this.m_pigeon = new PigeonIMU(Constants.Drivetrain.PIGEON_IMU_PORT);

        this.configShuffleboard();
    }

    public void configShuffleboard() {
        SendableRegistry.add(this.m_drive, "DifferentialDrive");

        ShuffleboardTab tab = Shuffleboard.getTab("Drivetrain");
        tab.add(this);
        tab.add(this.m_drive);
        tab.addNumber("Left Encoder Position", this::getLeftEncoderPosition);
        tab.addNumber("Left Encoder Velocity", this::getLeftEncoderVelocity);
        tab.addNumber("Right Encoder Position", this::getRightEncoderPosition);
        tab.addNumber("Right Encoder Velocity", this::getRightEncoderVelocity);
        tab.addNumber("Left Spark Master Current", m_sparkLeft1Master::getOutputCurrent);
        tab.addNumber("Right Spark Master Current", m_sparkRight1Master::getOutputCurrent);
    }

    public void curvatureDrive(double throttle, double steering, boolean isQuickTurn) {
        m_drive.curvatureDrive(throttle, steering, isQuickTurn);
    }

    /**
     * Reset encoders
     * @param posLeft left encoder position
     * @param posRight right encoder position
     */
    public void resetEncoders(double posLeft, double posRight) {
        m_encoderRight.setPosition(posLeft);
        m_encoderLeft.setPosition(posRight);
    }

    public double getLeftEncoderPosition() {
        return m_encoderLeft.getPosition();
    }

    public double getRightEncoderPosition() {
        return m_encoderRight.getPosition();
    }

    public double getLeftEncoderVelocity() {
        return m_encoderLeft.getVelocity();
    }

    public double getRightEncoderVelocity() {
        return m_encoderRight.getVelocity();
    }

    public double getHeading() {
        return m_pigeon.getFusedHeading();
    }

    /**
     * Reset heading of gyro
     * @param heading degrees
     */
    public void setHeading(double heading) {
        m_pigeon.setFusedHeading(heading);
    }

    @Override
    public void periodic() {
    }

    @Override
    public void tunePeriodic() {

    }

}
