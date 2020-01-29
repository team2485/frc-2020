package frc.team2485.robot.commands;

import com.ctre.phoenix.sensors.PigeonIMU;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team2485.WarlordsLib.control.WL_PIDController;
import frc.team2485.WarlordsLib.robotConfigs.RobotConfigs;
import frc.team2485.robot.subsystems.Turret;

import java.util.function.DoubleSupplier;

public class TurretPositionAlign extends CommandBase {

    private WL_PIDController pid;

    private Turret turret;

    private DoubleSupplier doubleSupplier;

    private PigeonIMU pigeon;

    public TurretPositionAlign(Turret turret, DoubleSupplier doubleSupplier) {
        this.turret = turret;
        addRequirements(turret);
        this.doubleSupplier = doubleSupplier;

        this.pid = new WL_PIDController();
        pid.setTolerance(2);

        pigeon = new PigeonIMU(0);

        RobotConfigs.getInstance().addConfigurable("Turret PID", pid);
    }

    @Override
    public void initialize() {
        pigeon.setFusedHeading(0);
    }

    @Override
    public void execute() {
        double output = pid.calculate(turret.getEncoderPosition() - pigeon.getFusedHeading(), doubleSupplier.getAsDouble());


        SmartDashboard.putNumber("pid measurement", turret.getEncoderPosition() - pigeon.getFusedHeading());
        SmartDashboard.putNumber("pid setpoint", doubleSupplier.getAsDouble());
        SmartDashboard.putNumber("Pigeon imu", pigeon.getFusedHeading());
        turret.setPWM(output);
    }
}
