package frc.team2485.robot.commands.shooter;

import edu.wpi.first.wpilibj.smartdashboard.SendableRegistry;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team2485.WarlordsLib.control.WL_PIDController;
import frc.team2485.WarlordsLib.robotConfigs.RobotConfigs;
import frc.team2485.robot.subsystems.Flywheels;

import java.util.function.DoubleSupplier;

public class SetFlywheels extends CommandBase {
    private Flywheels m_flywheels;
    private DoubleSupplier m_velocity;
    private double m_spinFactor;

    public SetFlywheels(Flywheels flywheels, DoubleSupplier velocity) {
        this(flywheels, velocity, 1);
    }

    public SetFlywheels(Flywheels flywheels, DoubleSupplier velocity, double spinFactor) {
        addRequirements(flywheels);
        this.m_flywheels = flywheels;
        this.m_velocity = velocity;
        this.m_spinFactor = spinFactor;

    }


    public void execute() {
        m_flywheels.setVelocity(m_velocity.getAsDouble(), m_spinFactor);
    }



}
