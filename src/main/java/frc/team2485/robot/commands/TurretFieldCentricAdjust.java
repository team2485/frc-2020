package frc.team2485.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpiutil.math.MathUtil;
import frc.team2485.robot.subsystems.Turret;

import java.util.function.DoubleSupplier;

public class TurretFieldCentricAdjust extends CommandBase {

    private Turret m_turret;

    private DoubleSupplier m_headingIncrement;
    private DoubleSupplier m_headingSource;

    private double m_setpoint;

    public TurretFieldCentricAdjust(Turret turret, DoubleSupplier headingIncrement, DoubleSupplier headingSource) {
        addRequirements(turret);

        this.m_turret = turret;

        this.m_headingIncrement = headingIncrement;
        this.m_headingSource = headingSource;
    }

    public void initialize() {
        resetSetpoint();
    }

    public void resetSetpoint() {
        m_setpoint = m_turret.getEncoderPosition() + m_headingSource.getAsDouble();
    }

    public double getSetpoint() {
        return m_setpoint;
    }

    @Override
    public void execute() {
//        if (m_headingIncrement.getAsDouble() == 0) {
//            resetSetpoint();
//        }
        m_setpoint += m_headingIncrement.getAsDouble();
        MathUtil.clamp(m_setpoint, -175, 175);
        m_turret.calculatePosition(m_setpoint - m_headingSource.getAsDouble());
    }
}
