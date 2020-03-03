package frc.team2485.robot.subsystems;

import com.ctre.phoenix.sensors.PigeonIMU;
import com.revrobotics.CANEncoder;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableRegistry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team2485.WarlordsLib.RampRate;
import frc.team2485.WarlordsLib.motorcontrol.WL_SparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team2485.WarlordsLib.robotConfigs.RobotConfigs;
import frc.team2485.WarlordsLib.sensors.SparkMaxAlternateEncoder;
import frc.team2485.robot.Constants;


public class Drivetrain extends SubsystemBase  {

    private DifferentialDrive m_drive;
    public static DifferentialDriveOdometry m_odometry;

    private WL_SparkMax m_sparkLeft1Master;
    private WL_SparkMax m_sparkLeft2;
    private WL_SparkMax m_sparkLeft3;

    private WL_SparkMax m_sparkRight1Master;
    private WL_SparkMax m_sparkRight2;
    private WL_SparkMax m_sparkRight3;

    private SparkMaxAlternateEncoder m_encoderLeft;
    private SparkMaxAlternateEncoder m_encoderRight;

    private RampRate m_throttleRamp;

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


        this.m_pigeon = new PigeonIMU(Constants.Drivetrain.PIGEON_IMU_PORT);
        this.m_drive = new DifferentialDrive(m_sparkLeft1Master, m_sparkRight1Master);
        this.m_odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(m_pigeon.getFusedHeading()));

        this.m_encoderLeft = new SparkMaxAlternateEncoder(Constants.Drivetrain.LEFT_ENCODER_SPARK, Constants.Drivetrain.ENCODER_CPR);
        this.m_encoderRight = new SparkMaxAlternateEncoder(Constants.Drivetrain.RIGHT_ENCODER_SPARK, Constants.Drivetrain.ENCODER_CPR);

        this.m_encoderRight.setInverted(true);


        this.m_encoderLeft.setDistancePerRevolution(Constants.Drivetrain.DISTANCE_PER_REVOLUTION);
        this.m_encoderRight.setDistancePerRevolution(Constants.Drivetrain.DISTANCE_PER_REVOLUTION);

        this.m_throttleRamp = new RampRate();

        SendableRegistry.add(this.m_drive, "DifferentialDrive");

        RobotConfigs.getInstance().addConfigurable("drivetrainThrottleRamp", m_throttleRamp);

        this.addToShuffleboard();
    }

    public void addToShuffleboard() {
        ShuffleboardTab tab = Shuffleboard.getTab("Drivetrain");
        tab.add(this);
        tab.add(this.m_drive);

        tab.add("throttle ramp", this.m_throttleRamp);
        tab.addNumber("Left Encoder Position", this::getLeftEncoderPosition);
        tab.addNumber("Left Encoder Velocity", this::getLeftEncoderVelocity);
        tab.addNumber("Right Encoder Position", this::getRightEncoderPosition);
        tab.addNumber("Right Encoder Velocity", this::getRightEncoderVelocity);
        tab.addNumber("Left Spark Master Current", m_sparkLeft1Master::getOutputCurrent);
        tab.addNumber("Right Spark Master Current", m_sparkRight1Master::getOutputCurrent);
    }

    public void curvatureDrive(double throttle, double steering, boolean isQuickTurn) {
        double throttleNextValue = m_throttleRamp.getNextValue(throttle);
        m_drive.curvatureDrive(throttleNextValue, steering, isQuickTurn);
        m_throttleRamp.setLastValue(throttleNextValue);
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

    /**
     * Reset heading of gyro
     * @param heading degrees
     */
    public void setHeading(double heading) {
        m_pigeon.setFusedHeading(heading);
    }

    public double getHeading() {
        return -m_pigeon.getFusedHeading();
    }

    public void driveVolts(double leftVolts, double rightVolts) {
        m_sparkLeft1Master.setVoltage(leftVolts);
        m_sparkRight1Master.setVoltage(-rightVolts);
        m_drive.feed();
    }

    public CANEncoder getIntakeArmEncoder() {
        return m_sparkLeft3.getEncoder();
    }

    public Pose2d getPose() {
        return m_odometry.getPoseMeters();
    }

    public DifferentialDriveWheelSpeeds getWheelSpeeds() {
        return new DifferentialDriveWheelSpeeds(m_encoderLeft.getVelocity(), m_encoderRight.getVelocity());
    }

    @Override
    public void periodic() {
    }


}
