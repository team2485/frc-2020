package frc.team2485.robot.subsystems;

import com.ctre.phoenix.sensors.PigeonIMU;
import com.revrobotics.AlternateEncoderType;
import com.revrobotics.CANEncoder;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SendableRegistry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team2485.WarlordsLib.RampRate;
import frc.team2485.WarlordsLib.motorcontrol.WL_SparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team2485.WarlordsLib.robotConfigs.RobotConfigs;
import frc.team2485.WarlordsLib.sensors.SparkMaxAlternateEncoder;
import frc.team2485.robot.Constants;
import com.revrobotics.CANSparkMax;


public class Drivetrain extends SubsystemBase  {

    private DifferentialDrive m_drive;
    public static DifferentialDriveOdometry m_odometry;

    private WL_SparkMax m_sparkLeft1Master;
    private WL_SparkMax m_sparkLeft2;
    private WL_SparkMax m_sparkLeft3;

    private WL_SparkMax m_sparkRight1Master;
    private WL_SparkMax m_sparkRight2;
    private WL_SparkMax m_sparkRight3;

    private CANEncoder m_encoderLeft;
    private CANEncoder m_encoderRight;

    private RampRate m_throttleRamp;

    private PigeonIMU m_pigeon;

    private SendableChooser<CANSparkMax.IdleMode> m_dashboardChooser;

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

        this.m_encoderLeft = m_sparkLeft1Master.getAlternateEncoder(AlternateEncoderType.kQuadrature, 500);

        this.m_encoderRight = m_sparkRight1Master.getAlternateEncoder(AlternateEncoderType.kQuadrature, 500);
        this.m_encoderRight.setInverted(true);



        this.m_encoderLeft.setPositionConversionFactor(Constants.Drivetrain.DISTANCE_PER_REVOLUTION);
        this.m_encoderRight.setPositionConversionFactor(Constants.Drivetrain.DISTANCE_PER_REVOLUTION);

        this.m_throttleRamp = new RampRate();
        this.m_throttleRamp.setRampRates(0.4, 0.1);


        
       
        
        m_dashboardChooser = new SendableChooser<CANSparkMax.IdleMode> ();
        m_dashboardChooser.addOption("Brake", CANSparkMax.IdleMode.kBrake);
        m_dashboardChooser.addOption("Coast", CANSparkMax.IdleMode.kCoast);
        m_dashboardChooser.setDefaultOption("Coast", CANSparkMax.IdleMode.kCoast);
        setIdleMode(m_dashboardChooser.getSelected());

        SendableRegistry.add(this.m_drive, "DifferentialDrive");
        
        m_odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getHeading()));

        //RobotConfigs.getInstance().addConfigurable("drivetrainThrottleRamp", m_throttleRamp);
        this.addToShuffleboard();

    }

    public void addToShuffleboard() {
        ShuffleboardTab tab = Shuffleboard.getTab("Drivetrain");
        tab.add(this);
        tab.add(this.m_drive);
        tab.add(m_dashboardChooser);
        tab.addNumber("Left PWM", m_sparkLeft1Master::getAppliedOutput);
        tab.addNumber("Right PWM", m_sparkRight1Master::getAppliedOutput);
       // tab.add("throttle ramp", this.m_throttleRamp);
        tab.addNumber("Left Encoder Position", this::getLeftEncoderPosition);
        tab.addNumber("Left Encoder Velocity", this::getLeftEncoderVelocity);
        tab.addNumber("Right Encoder Position", this::getRightEncoderPosition);
        tab.addNumber("Right Encoder Velocity", this::getRightEncoderVelocity);
        tab.addNumber("Left Spark Master Current", m_sparkLeft1Master::getOutputCurrent);
        tab.addNumber("Right Spark Master Current", m_sparkRight1Master::getOutputCurrent);

    }

    public void setIdleMode(CANSparkMax.IdleMode mode) {
        System.out.println(mode);
        m_sparkLeft1Master.setIdleMode(mode);
        m_sparkLeft2.setIdleMode(mode);
        m_sparkLeft3.setIdleMode(mode);
        m_sparkRight1Master.setIdleMode(mode);
        m_sparkRight2.setIdleMode(mode);
        m_sparkRight3.setIdleMode(mode);
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

    public void resetEncoders() {
        m_encoderRight.setPosition(0);
        m_encoderLeft.setPosition(0);
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
        return m_pigeon.getFusedHeading();
    }

    public void driveVolts(double leftVolts, double rightVolts) {
        m_sparkLeft1Master.setVoltage(leftVolts);
        m_sparkRight1Master.setVoltage(-rightVolts);
        m_drive.feed();
    }

    public CANEncoder getIntakeArmEncoder() {
        return m_sparkLeft3.getAlternateEncoder();
    }

    public Pose2d getPose() {
        return m_odometry.getPoseMeters();
    }

    public void resetOdometry(Pose2d pose) {
        resetEncoders();
        m_odometry.resetPosition(pose, Rotation2d.fromDegrees(getHeading()));
      }

    public DifferentialDriveWheelSpeeds getWheelSpeeds() {
        return new DifferentialDriveWheelSpeeds(m_encoderLeft.getVelocity(), m_encoderRight.getVelocity());
    }

    @Override
    public void periodic() {
       m_odometry.update(Rotation2d.fromDegrees(getHeading()), m_encoderLeft.getPosition(), m_encoderRight.getPosition());
       setIdleMode(m_dashboardChooser.getSelected());
    }


}
