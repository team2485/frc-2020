package frc.team2485.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team2485.robot.subsystems.Turret;

import java.util.function.DoubleSupplier;

public class TurretSetAngle extends CommandBase {

    private Turret m_turret;

    private DoubleSupplier m_angleSetpoint;

    private boolean m_finishWhenAligned;

    /**
     * Set angle at a particular heading
     * @param turret
     * @param angleSetpoint source for the set point for the m_turret angle
     */
    public TurretSetAngle(Turret turret, DoubleSupplier angleSetpoint) {
        addRequirements(turret);

        this.m_turret = turret;

        this.m_angleSetpoint = angleSetpoint;

        this.m_finishWhenAligned = false;
    }

    /**
     * Set angle at a particular heading
     * @param turret turret subsystem reference
     * @param target angle target
     * @param finishWhenAligned finish this command when it ultimately aligns.
     */
    public TurretSetAngle(Turret turret, double target, boolean finishWhenAligned) {
        this(turret, () -> target);

        this.m_finishWhenAligned = finishWhenAligned;
    }

    @Override
    public void initialize() {
        m_turret.resetPIDs();
    }

    @Override
    public void execute() {
        m_turret.runPositionPID(m_angleSetpoint.getAsDouble());
    }

    @Override
    public boolean isFinished() {
        if (m_finishWhenAligned) {
            return m_turret.atPositionSetpoint();
        }
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        m_turret.setPWM(0);
    }
}
