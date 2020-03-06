package frc.team2485.robot.commandgroups;

import edu.wpi.first.wpilibj2.command.*;
import frc.team2485.robot.Constants;
import frc.team2485.robot.commands.HighMagazineIncrement;
import frc.team2485.robot.subsystems.BallCounter;
import frc.team2485.robot.subsystems.HighMagazine;
import frc.team2485.robot.subsystems.LowMagazine;

public class IntakeBalls extends ParallelCommandGroup {

    private LowMagazine m_lowMagazine;
    private HighMagazine m_highMagazine;
    private BallCounter m_ballCounter;

    private boolean m_init = false;

    private boolean m_incrementFinished;

    public IntakeBalls(LowMagazine lowMagazine, HighMagazine highMagazine, BallCounter ballCounter) {
        m_lowMagazine = lowMagazine;
        m_highMagazine = highMagazine;
        m_ballCounter = ballCounter;

        this.addCommands(
                new RunCommand(
                        () -> {
                            if (!( // run until:
                                    (m_ballCounter.getNumBallsHigh() >= Constants.Magazine.HIGH_MAGAZINE_BALL_CAPACITY
                                            || (m_ballCounter.getNumBallsHigh() >= 3 && m_ballCounter.exitIRHasBall()))
                                            && m_ballCounter.transferIRHasBall()
                            )) {
                                m_lowMagazine.runVelocityPID(Constants.Magazine.Setpoints.LOW_INTAKE_VELOCITY);
                            } else {
                                m_lowMagazine.setPWM(0);
                            }
                        }, lowMagazine
                ),
                new RunCommand(
                        () -> {
                            if (m_ballCounter.transferIRHasBall()
                                    && (m_ballCounter.getNumBallsHigh() < Constants.Magazine.HIGH_MAGAZINE_BALL_CAPACITY)
                                    && (m_incrementFinished || m_init)) {
                                m_incrementFinished = false;
                                new HighMagazineIncrement(highMagazine, Constants.Magazine.Setpoints.HIGH_INDEX_BY_ONE_POS)
                                        .andThen(new InstantCommand(() -> m_incrementFinished = true))
                                        .schedule();
                                m_init = false;
                            }
                        }, highMagazine
                )
        );
    }

    @Override
    public void initialize() {
        m_lowMagazine.resetPIDs();
        m_highMagazine.resetPIDs();
        m_init = true;
        m_incrementFinished = true;
    }

    @Override
    public void end(boolean interrupted) {
        m_lowMagazine.setPWM(0);
        m_highMagazine.setPWM(0);
    }
}
