package frc.team2485.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team2485.robot.subsystems.Flywheels;

import java.util.function.DoubleSupplier;

public class SetFlywheels extends CommandBase {

    private Flywheels m_flywheels;
    private DoubleSupplier m_leftVelocity;
    private DoubleSupplier m_rightVelocity;

    public SetFlywheels(Flywheels flywheels, double velocity) {
        this(flywheels, () -> velocity);
    }

    public SetFlywheels(Flywheels flywheels, DoubleSupplier leftVelocity, DoubleSupplier rightVelocity) {
        addRequirements(flywheels);
        this.m_flywheels = flywheels;
        this.m_leftVelocity = leftVelocity;
        this.m_rightVelocity = rightVelocity;
    }

    public SetFlywheels(Flywheels flywheels, DoubleSupplier velocity) {
        this(flywheels, velocity, velocity);
    }

    @Override
    public void initialize() {
        m_flywheels.resetPIDs();
    }

    public void execute() {
        m_flywheels.setVelocity(m_leftVelocity.getAsDouble(), m_rightVelocity.getAsDouble());
    }

    @Override
    public void end(boolean interrupted) {
        m_flywheels.setPWM(0);
    }
}
