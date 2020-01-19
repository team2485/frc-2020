package frc.team2485.robot.subsystems;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team2485.WarlordsLib.WL_DifferentialDrive;
import frc.team2485.WarlordsLib.motorcontrol.WL_SparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team2485.robot.Constants;


public class Drivetrain extends SubsystemBase {

    private DifferentialDrive m_drive;

    private WL_SparkMax m_sparkLeft1Master;
    private WL_SparkMax m_sparkLeft2;
    private WL_SparkMax m_sparkLeft3;

    private WL_SparkMax m_sparkRight1Master;
    private WL_SparkMax m_sparkRight2;
    private WL_SparkMax m_sparkRight3;

    public Drivetrain() {
        this.m_sparkLeft1Master = new WL_SparkMax(Constants.Drivetrain.SPARK_LEFT_PORT_MASTER);
        this.m_sparkLeft2 = new WL_SparkMax(Constants.Drivetrain.SPARK_LEFT_PORT_SLAVE_2);
        this.m_sparkLeft3 = new WL_SparkMax(Constants.Drivetrain.SPARK_LEFT_PORT_SLAVE_3);

        this.m_sparkRight1Master = new WL_SparkMax(Constants.Drivetrain.SPARK_RIGHT_PORT_MASTER);
        this.m_sparkRight2 = new WL_SparkMax(Constants.Drivetrain.SPARK_RIGHT_PORT_SLAVE_2);
        this.m_sparkRight3 = new WL_SparkMax(Constants.Drivetrain.SPARK_RIGHT_PORT_SLAVE_3);

        m_sparkLeft1Master.setFollowers(m_sparkLeft2, m_sparkLeft3);
        m_sparkRight1Master.setFollowers(m_sparkRight2, m_sparkRight3);

        m_drive = new WL_DifferentialDrive(m_sparkLeft1Master, m_sparkRight1Master);

        SmartDashboard.putData(this);

    }

    public void curvatureDrive(double throttle, double steering, boolean isQuickTurn) {
        m_drive.curvatureDrive(throttle, steering, isQuickTurn);
    }

}
