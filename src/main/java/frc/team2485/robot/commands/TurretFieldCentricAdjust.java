package frc.team2485.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpiutil.math.MathUtil;
import frc.team2485.robot.subsystems.Turret;

import java.util.function.DoubleSupplier;

public class TurretFieldCentricAdjust extends CommandBase {

    private Turret m_turret;

    private DoubleSupplier m_headingIncrement;
    private DoubleSupplier m_headingSource;

    private double m_setpoint;

    /**
     * Command for turret field-centric adjust
     * @param turret turret subsystem instance
     * @param headingIncrement this value is used to increment the setpoint.
     * @param headingSource this should be a reference to a method that gets the robot heading.
     */
    public TurretFieldCentricAdjust(Turret turret, DoubleSupplier headingIncrement, DoubleSupplier headingSource) {
        addRequirements(turret);

        this.m_turret = turret;

        this.m_headingIncrement = headingIncrement;
        this.m_headingSource = headingSource;
    }

    public void initialize() {
        resetSetpoint();
        m_turret.resetPID();
    }

    /**
     * Sets the setpoint to the turret's current field-centric heading.
     */
    public void resetSetpoint() {
        m_setpoint = m_turret.getEncoderPosition() + m_headingSource.getAsDouble();
    }

    public double getSetpoint() {
        return m_setpoint;
    }

    @Override
    public void execute() {
        m_setpoint += m_headingIncrement.getAsDouble();

        // Push the setpoint if we reach out hardstops
        if (m_setpoint - m_headingSource.getAsDouble() > m_turret.getMaxAngle()) {
            m_setpoint = m_turret.getMaxAngle() + m_headingSource.getAsDouble();
        } else if (m_setpoint - m_headingSource.getAsDouble() < m_turret.getMinAngle()) {
            m_setpoint = m_turret.getMinAngle() + m_headingSource.getAsDouble();
        }

        m_turret.runPID(m_setpoint - m_headingSource.getAsDouble());
    }
}
