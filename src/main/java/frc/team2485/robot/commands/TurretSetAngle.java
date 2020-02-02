package frc.team2485.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team2485.robot.subsystems.Turret;

import java.util.function.DoubleSupplier;

public class TurretSetAngle extends CommandBase {

    private Turret m_turret;

    private DoubleSupplier m_headingSetpoint;

    private boolean m_isAtTarget;

    private boolean m_finishWhenAligned;

    /**
     * Set angle at a particular heading
     * @param turret
     * @param headingSetpoint source for the set point for the m_turret angle
     */
    public TurretSetAngle(Turret turret, DoubleSupplier headingSetpoint) {
        addRequirements(turret);

        this.m_turret = turret;

        this.m_headingSetpoint = headingSetpoint;

        this.m_finishWhenAligned = false;
    }

    public TurretSetAngle(Turret turret, double target, boolean finishWhenAligned) {
        this(turret, () -> target);

        this.m_finishWhenAligned = finishWhenAligned;
    }

    @Override
    public void execute() {
        this.m_isAtTarget = m_turret.usePositionPID(m_headingSetpoint.getAsDouble());
    }

    public boolean isM_isAtTarget() {
        return this.m_isAtTarget;
    }

    @Override
    public boolean isFinished() {
        if (m_finishWhenAligned) {
            return this.isM_isAtTarget();
        }

        return false;
    }
}
