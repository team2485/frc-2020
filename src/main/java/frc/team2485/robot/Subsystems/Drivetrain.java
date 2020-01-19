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

    private SparkMaxAlternateEncoderWrapper m_encoderLeft;
    private SparkMaxAlternateEncoderWrapper m_encoderRight;

    public Drivetrain() {
        this.m_sparkLeft1Master = new WL_SparkMax(Constants.Drivetrain.SPARK_LEFT_PORT_MASTER);
        this.m_sparkLeft2 = new WL_SparkMax(Constants.Drivetrain.SPARK_LEFT_PORT_SLAVE_2);
        this.m_sparkLeft3 = new WL_SparkMax(Constants.Drivetrain.SPARK_LEFT_PORT_SLAVE_3);

        this.m_sparkRight1Master = new WL_SparkMax(Constants.Drivetrain.SPARK_RIGHT_PORT_MASTER);
        this.m_sparkRight2 = new WL_SparkMax(Constants.Drivetrain.SPARK_RIGHT_PORT_SLAVE_2);
        this.m_sparkRight3 = new WL_SparkMax(Constants.Drivetrain.SPARK_RIGHT_PORT_SLAVE_3);

        m_sparkLeft1Master.setFollowers(m_sparkLeft2, m_sparkLeft3);
        m_sparkRight1Master.setFollowers(m_sparkRight2, m_sparkRight3);

        m_encoderLeft = new SparkMaxAlternateEncoderWrapper(Constants.Drivetrain.SPARK_LEFT_ENCODER, Constants.Drivetrain.ENCODER_CPR);
        m_encoderRight = new SparkMaxAlternateEncoderWrapper(Constants.Drivetrain.SPARK_RIGHT_ENCODER, Constants.Drivetrain.ENCODER_CPR);

        m_drive = new WL_DifferentialDrive(m_sparkLeft1Master, m_sparkRight1Master);

        resetEncoders(0,0);

        m_encoderRight.getEncoder().setInverted(true);

        SmartDashboard.putData(this);

        SendableRegistry.add(m_drive, "DifferentialDrive");
    }

    public void curvatureDrive(double throttle, double steering, boolean isQuickTurn) {
        m_drive.curvatureDrive(throttle, steering, isQuickTurn);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Left encoder", m_encoderLeft.getPosition());
        SmartDashboard.putNumber("Right encoder", m_encoderRight.getPosition());
    }

    public void resetEncoders(double posLeft, double posRight) {
        m_encoderRight.setPosition(posLeft);
        m_encoderLeft.setPosition(posRight);
    }

}
