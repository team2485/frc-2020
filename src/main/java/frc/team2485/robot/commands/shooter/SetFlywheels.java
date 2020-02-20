package frc.team2485.robot.commands.shooter;

import edu.wpi.first.wpilibj.smartdashboard.SendableRegistry;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team2485.WarlordsLib.control.WL_PIDController;
import frc.team2485.WarlordsLib.robotConfigs.RobotConfigs;
import frc.team2485.robot.subsystems.Flywheels;

public class SetFlywheels extends CommandBase {
    private Flywheels m_flywheels;
    private WL_PIDController m_pidController;
    private double m_velocity;
    private double m_spinFactor;

    public SetFlywheels(Flywheels flywheels, double velocity) {
        this(flywheels, velocity, 1);
    }

    public SetFlywheels(Flywheels flywheels, double velocity, double spinFactor) {
        addRequirements(flywheels);
        this.m_flywheels = flywheels;
        this.m_velocity = velocity;
        this.m_pidController = new WL_PIDController();
        this.m_spinFactor = spinFactor;

        SendableRegistry.add(m_pidController, "Flywheels Velocity Controller");
        RobotConfigs.getInstance().addConfigurable("flywheelsVelocityController", m_pidController);
    }


    public void initialize() {
        m_pidController.setSetpoint(m_velocity);
    }



}
