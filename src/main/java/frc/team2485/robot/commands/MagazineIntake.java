package frc.team2485.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team2485.robot.subsystems.Magazine;

public class MagazineIntake extends CommandBase {
    private Magazine m_magazine;
    private boolean ball1Caught;

    public MagazineIntake(Magazine magazine) {
        addRequirements(magazine);
        m_magazine = magazine;
        ball1Caught = false;
    }

    public void initialize() {
        m_magazine.runLowBelt();
    }

    public void execute() {
        m_magazine.updateBalls();
        if(m_magazine.getBallsContained() == 1 ) { //perhaps add magazine get Endbelt1?
            m_magazine.runHighBelt();
        } else if(m_magazine.getBallsContained() >= 1 && m_magazine.getBallsContained() < 5 && !m_magazine.getEndBelt2()) { //last boolean checks if ball at top, if so, we need to stop
            if(m_magazine.getEndBelt1()) {
                m_magazine.indexHighBelt();
            }
        } else {
            m_magazine.stopLowBelt();
            end(false);
        }

        if(m_magazine.getStartBelt2() && !ball1Caught) {
            ball1Caught = true;
            m_magazine.stopHighBelt();
        }
    }

    /**
     *
     * @param interrupted for this command, interrupted signals that less than 5 balls have been intaken, called by Intake command's end.
     */
    public void end(boolean interrupted) {
        if(interrupted) {
            for(int i = 0; i < 5 - m_magazine.getBallsContained(); i++) {
                m_magazine.indexHighBelt();
            }
        }
    }


}
