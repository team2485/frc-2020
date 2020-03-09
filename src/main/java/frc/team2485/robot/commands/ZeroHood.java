package frc.team2485.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team2485.robot.Constants;
import frc.team2485.robot.subsystems.Hood;

public class ZeroHood extends CommandBase {

    private Hood m_hood;

    private double m_velocity;

    public ZeroHood(Hood hood, double velocity) {
        this.m_hood = hood;
        this.m_velocity = velocity;
    }

    @Override
    public void execute() {
        m_hood.runVelocityPID(m_velocity);
    }

    @Override
    public boolean isFinished() {
        return m_hood.getReverseLimitSwitch();
    }

    @Override
    public void end(boolean interrupted) {
        m_hood.setPWM(0);
        m_hood.setEncoderPosition(Constants.Hood.HOOD_TOP_POSITION_DEG);
    }
}
