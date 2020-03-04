package frc.team2485.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team2485.robot.subsystems.BallCounter;
import frc.team2485.robot.subsystems.HighMagazine;

public class IncrementHighMagazine extends CommandBase {

    private HighMagazine m_highMagazine;

    private double m_distanceSetpoint;

    private double m_distance;

    public IncrementHighMagazine(HighMagazine magazine, double distance) {
        m_highMagazine = magazine;
        m_distance = distance;
    }

    @Override
    public void initialize() {
        m_highMagazine.resetPIDs();
        m_distanceSetpoint = m_highMagazine.getEncoderPosition() + m_distance;
    }

    @Override
    public void execute() {
        m_highMagazine.runPositionPID(m_distanceSetpoint);
    }

    @Override
    public boolean isFinished() {
        return m_highMagazine.atPositionSetpoint();
    }
}
