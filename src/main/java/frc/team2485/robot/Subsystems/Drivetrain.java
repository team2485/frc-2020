package frc.team2485.robot.Subsystems;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.team2485.WarlordsLib.WL_DifferentialDrive;
import frc.team2485.WarlordsLib.motorcontrol.WL_SparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;



public class Drivetrain extends SubsystemBase {

    private DifferentialDrive drive;


    private WL_SparkMax _sparkLeft1Master;
    private WL_SparkMax _sparkLeft2;
    private WL_SparkMax _sparkLeft3;

    private WL_SparkMax _sparkRight1Master;
    private WL_SparkMax _sparkRight2;
    private WL_SparkMax _sparkRight3;

    public Drivetrain() {
        this._sparkLeft1Master = new WL_SparkMax(0);
        this._sparkLeft2 = new WL_SparkMax(0);
        this._sparkLeft3 = new WL_SparkMax(0);

        this._sparkRight1Master = new WL_SparkMax(0);
        this._sparkRight2 = new WL_SparkMax(0);
        this._sparkRight3 = new WL_SparkMax(0);

        _sparkLeft1Master.setFollowers(_sparkLeft2, _sparkLeft3);
        _sparkRight1Master.setFollowers(_sparkRight2, _sparkRight3);

        drive = new WL_DifferentialDrive(_sparkLeft1Master, _sparkRight1Master);
    }

    public void curvatureDrive(double throttle, double steering, boolean isQuickTurn) {
        drive.curvatureDrive(throttle, steering, isQuickTurn);
    }

}
