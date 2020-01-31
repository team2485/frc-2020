package frc.team2485.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team2485.robot.subsystems.Magazine;

public class MagazineOuttake extends CommandBase {
    private Magazine m_magazine;
    private double m_outtakeVelocity;
    private boolean m_ball5Caught;
    //delay between outtakes of each bll in milliseconds
    private double m_delay;
    private double m_executeCounter;

    public MagazineOuttake (Magazine magazine, double outtakeVelocity, double delay) {
        addRequirements(magazine);
        m_magazine = magazine;
        m_outtakeVelocity = outtakeVelocity;
        m_delay = delay;
        m_executeCounter = 0;
    }
    public void initialize() {
        m_magazine.runOuttakeWheels(m_outtakeVelocity);
    }

    public void execute() {
        m_executeCounter++;

        if((m_executeCounter*20) % m_delay == 0) {
            if(m_magazine.getBallsContained() == 5) {
                m_magazine.indexHighBelt();
                m_magazine.runLowBelt();
                m_magazine.decreaseBallsContained();
            } else {
                m_magazine.indexHighBelt();
                m_magazine.decreaseBallsContained();
            }
        }
            //may not work. start belt 2 may pick up on ball 4, indexing may be too slow. may be smarter just to stop low belt when 3 balls
        if (m_magazine.getStartBelt2()) {
            m_ball5Caught = true;
            m_magazine.stopLowBelt();
        }
    }

    public boolean isFinished() {
        return m_magazine.getBallsContained() == 0;
    }

    public void end(boolean interrupted) {
        m_magazine.runOuttakeWheels(0);
    }

}
