package frc.team2485.robot.subsystems;

import com.revrobotics.AlternateEncoderType;
import com.revrobotics.CANEncoder;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SendableRegistry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team2485.WarlordsLib.WL_DifferentialDrive;
import frc.team2485.WarlordsLib.motorcontrol.WL_SparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team2485.WarlordsLib.sensors.SparkMaxAlternateEncoderWrapper;
import frc.team2485.robot.Constants;


public class Drivetrain extends SubsystemBase {

    private DifferentialDrive m_drive;

    private WL_SparkMax m_sparkLeft1Master;
    private WL_SparkMax m_sparkLeft2;
    private WL_SparkMax m_sparkLeft3;

    private WL_SparkMax m_sparkRight1Master;
    private WL_SparkMax m_sparkRight2;
    private WL_SparkMax m_sparkRight3;

    private static SparkMaxAlternateEncoderWrapper m_encoderLeft;
    private static SparkMaxAlternateEncoderWrapper m_encoderRight;

    public static DifferentialDriveOdometry m_odometry;
    public static PigeonIMU gyro;

    public static Translation2d translationValue;
    public static Rotation2d rotationValue;

    public Drivetrain() {
        this.m_sparkLeft1Master = new WL_SparkMax(Constants.Drivetrain.SPARK_LEFT_PORT_MASTER);
        this.m_sparkLeft2 = new WL_SparkMax(Constants.Drivetrain.SPARK_LEFT_PORT_SLAVE_2);
        this.m_sparkLeft3 = new WL_SparkMax(Constants.Drivetrain.SPARK_LEFT_PORT_SLAVE_3);

        this.m_sparkRight1Master = new WL_SparkMax(Constants.Drivetrain.SPARK_RIGHT_PORT_MASTER);
        this.m_sparkRight2 = new WL_SparkMax(Constants.Drivetrain.SPARK_RIGHT_PORT_SLAVE_2);
        this.m_sparkRight3 = new WL_SparkMax(Constants.Drivetrain.SPARK_RIGHT_PORT_SLAVE_3);

        this.m_sparkLeft1Master.setSmartCurrentLimit(Constants.Drivetrain.CURRENT_LIMIT);
        this.m_sparkRight1Master.setSmartCurrentLimit(Constants.Drivetrain.CURRENT_LIMIT);

        this.m_sparkLeft1Master.setFollowers(m_sparkLeft2, m_sparkLeft3);
        this.m_sparkRight1Master.setFollowers(m_sparkRight2, m_sparkRight3);

        this.m_drive = new WL_DifferentialDrive(m_sparkLeft1Master, m_sparkRight1Master);

        m_odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(gyro.getFusedHeading()));
        gyro = new PigeonIMU(0);

        // 4x encoding so * 4
        this.m_encoderLeft = new SparkMaxAlternateEncoderWrapper(Constants.Drivetrain.SPARK_LEFT_ENCODER, Constants.Drivetrain.ENCODER_CPR * 4 );
        this.m_encoderRight = new SparkMaxAlternateEncoderWrapper(Constants.Drivetrain.SPARK_RIGHT_ENCODER, Constants.Drivetrain.ENCODER_CPR * 4);

        this.m_encoderRight.setInverted(true);

        this.m_encoderLeft.setDistancePerRevolution(2 * Math.PI * Constants.Drivetrain.WHEEL_RADIUS);
        this.m_encoderRight.setDistancePerRevolution(2 * Math.PI * Constants.Drivetrain.WHEEL_RADIUS);

        SmartDashboard.putData(this);
        SendableRegistry.add(this.m_drive, "DifferentialDrive");
    }

    public void curvatureDrive(double throttle, double steering, boolean isQuickTurn) {
        m_drive.curvatureDrive(throttle, steering, isQuickTurn);
    }

    /**
     * Reset encoders
     * @param posLeft left encoder position
     * @param posRight right encoder position
     */



    public void driveVolts(double leftVolts, double rightVolts) {
        m_sparkLeft1Master.setVoltage(leftVolts);
        m_sparkRight1Master.setVoltage(-rightVolts);
        m_drive.feed();
    }

    public void resetEncoders(double posLeft, double posRight) {
        m_encoderRight.setPosition(posLeft);
        m_encoderLeft.setPosition(posRight);
    }

    public Pose2d getPose() {
        return m_odometry.getPoseMeters();
    }

    public DifferentialDriveWheelSpeeds getWheelSpeeds() {
        return new DifferentialDriveWheelSpeeds(m_encoderLeft.getVelocity(), m_encoderRight.getVelocity());
    }


//  public void arcadeDrive(double fwd, double rot) {
//    m_drive.arcadeDrive(fwd, rot);
//  }

    public static void resetEncoders() {
        m_encoderLeft.setPosition(0);
        m_encoderRight.setPosition(0);
    }

    public static void resetOdometry(Pose2d newPose) {
        resetEncoders();
        m_odometry.resetPosition(newPose, new Rotation2d(0));
    }

    public double getAverageEncoderDistance() {
        return ((m_encoderLeft.getPosition() + m_encoderRight.getPosition()) / 2.0);
    }

    public SparkMaxAlternateEncoderWrapper getLeftEncoder() {
        return m_encoderLeft;
    }

    public SparkMaxAlternateEncoderWrapper getRightEncoder() {
        return m_encoderRight;
    }

    //some extra random stuff that we aren't using right now

//  public void setMaxOutput(double maxOutput) {
//    driveBase.setMaxOutput(maxOutput);
//  }

//  public void zeroHeading() {
//    gyro.setCompassAngle(0);
//  }

//  public double getHeading() {
//    return Math.IEEEremainder(gyro.getCompassHeading(), 360);
//  }

//    public double getTurnRate() {
//
//        return 8.0;
//        //return gyro.get
//    }

    @Override

    public void periodic() {

        m_odometry.update(Rotation2d.fromDegrees(gyro.getFusedHeading()), m_encoderLeft.getPosition(), m_encoderRight.getPosition());

        translationValue = m_odometry.getPoseMeters().getTranslation();

        rotationValue = m_odometry.getPoseMeters().getRotation();

    }



}
