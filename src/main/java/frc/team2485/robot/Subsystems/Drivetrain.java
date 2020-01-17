package frc.team2485.robot.Subsystems;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.team2485.WarlordsLib.WL_DifferentialDrive;
import frc.team2485.WarlordsLib.motorcontrol.WL_SparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;



public class Drivetrain extends SubsystemBase {

    private DifferentialDrive drive;


    private WL_SparkMax SparkLeft1Master;
    private WL_SparkMax SparkLeft2;
    private WL_SparkMax SparkLeft3;

    private WL_SparkMax SparkRight1Master;
    private WL_SparkMax SparkRight2;
    private WL_SparkMax SparkRight3;

    public Drivetrain() {
        this.SparkLeft1Master = new WL_SparkMax(0);
        this.SparkLeft2 = new WL_SparkMax(0);
        this.SparkLeft3 = new WL_SparkMax(0);

        this.SparkRight1Master = new WL_SparkMax(0);
        this.SparkRight2 = new WL_SparkMax(0);
        this.SparkRight3 = new WL_SparkMax(0);

        SparkLeft1Master.setFollowers(SparkLeft2, SparkLeft3);
        SparkRight1Master.setFollowers(SparkRight2, SparkRight3);

        drive = new WL_DifferentialDrive( SparkLeft1Master, SparkRight1Master);
    }

    public void curvatureDrive(double throttle, double steering, boolean isQuickTurn) {

        drive.curvatureDrive(throttle, steering, isQuickTurn);


    }
}
