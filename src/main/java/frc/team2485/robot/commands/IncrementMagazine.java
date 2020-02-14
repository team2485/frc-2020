package frc.team2485.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team2485.robot.Constants;
import frc.team2485.robot.subsystems.AbstractMagazinePart;

public class IncrementMagazine extends CommandBase {

    private AbstractMagazinePart m_magazine;

    private double m_distanceSetpoint;

    public IncrementMagazine(AbstractMagazinePart magazine, double distance) {
        m_magazine = magazine;
        m_distanceSetpoint = magazine.getEncoderPosition() + distance;
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
