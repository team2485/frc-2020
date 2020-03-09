package frc.team2485.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team2485.robot.subsystems.Hood;

import java.util.function.DoubleSupplier;

public class SetHood extends CommandBase {

    private Hood m_hood;
    private DoubleSupplier m_angle;
    private boolean m_finishWhenAtTarget;

    public SetHood(Hood hood, double angle) {
        this(hood, () -> angle, true);
    }

    public SetHood(Hood hood, DoubleSupplier angle) {
        this(hood, angle, true);
    }

    public SetHood(Hood hood, DoubleSupplier angle, boolean finishWhenAtTarget) {
        addRequirements(hood);

        this.m_hood = hood;
        this.m_angle = angle;
        this.m_finishWhenAtTarget = finishWhenAtTarget;

    }

    @Override
    public void initialize() {
        m_hood.resetPID();
    }

    @Override
    public void execute() {
        m_hood.runPositionPID(m_angle.getAsDouble());
    }

    @Override
    public boolean isFinished() {
        if (m_finishWhenAtTarget) {
            return m_hood.atPositionSetpoint();
        }
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        m_hood.setPWM(0);
    }
}
