package frc.team2485.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team2485.robot.subsystems.Flywheels;

import java.util.function.DoubleSupplier;

public class SetFlywheels extends CommandBase {

    private Flywheels m_flywheels;
    private DoubleSupplier m_leftVelocity;
    private DoubleSupplier m_rightVelocity;

    private boolean m_finishWhenAtTarget;

    public SetFlywheels(Flywheels flywheels, DoubleSupplier leftVelocity, DoubleSupplier rightVelocity) {
        addRequirements(flywheels);
        this.m_flywheels = flywheels;
        this.m_leftVelocity = leftVelocity;
        this.m_rightVelocity = rightVelocity;
        m_finishWhenAtTarget = true;
    }

    public SetFlywheels(Flywheels flywheels, DoubleSupplier velocity) {
        this(flywheels, velocity, velocity);
    }

    public SetFlywheels(Flywheels flywheels, DoubleSupplier velocity, boolean finishWhenAtTarget) {
        this(flywheels, velocity, velocity);
        m_finishWhenAtTarget = finishWhenAtTarget;
    }

    @Override
    public void initialize() {
        m_flywheels.resetPIDs();
    }

    public void execute() {
        System.out.println("left vel: " + m_leftVelocity.getAsDouble());
        System.out.println("right vel: " + m_rightVelocity.getAsDouble());
        m_flywheels.setVelocity(m_leftVelocity.getAsDouble(), m_rightVelocity.getAsDouble());
    }

    //if something's broken, this is it
    public boolean isFinished() {
        return false; 
        // if(m_finishWhenAtTarget) {
        //     return m_flywheels.atVelocitySetpoint();
        // } else {
        //     return false;
        // }
        
    }



}
