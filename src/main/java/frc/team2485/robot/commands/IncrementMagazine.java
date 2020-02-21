package frc.team2485.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team2485.robot.subsystems.AbstractMagazinePart;

public class IncrementMagazine extends CommandBase {

    private AbstractMagazinePart m_magazine;

    private double m_distanceSetpoint;

    private double m_distance;

    public IncrementMagazine(AbstractMagazinePart magazine, double distance) {
        m_magazine = magazine;
        m_distance = distance;
    }

    @Override
    public void initialize() {
        m_distanceSetpoint = m_magazine.getEncoderPosition() - m_distance;
    }

    @Override
    public void execute() {
        m_magazine.setPosition(m_distanceSetpoint);
    }

    @Override
    public boolean isFinished() {
        return m_magazine.atPositionSetpoint();
    }
}
