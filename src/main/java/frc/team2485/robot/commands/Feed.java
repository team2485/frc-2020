package frc.team2485.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.team2485.robot.Constants;
import frc.team2485.robot.subsystems.BallCounter;
import frc.team2485.robot.subsystems.Feeder;
import frc.team2485.robot.subsystems.HighMagazine;
import frc.team2485.robot.subsystems.LowMagazine;

public class Feed extends ParallelCommandGroup {

    private Feeder m_feeder;
    private HighMagazine m_highMagazine;
    private LowMagazine m_lowMagazine;
    private BallCounter m_ballCounter;

    public Feed(Feeder feeder, HighMagazine highMagazine, LowMagazine lowMagazine, BallCounter ballCounter, int numFeeds) {
        this.m_feeder = feeder;
        this.m_highMagazine = highMagazine;
        this.m_lowMagazine = lowMagazine;
        this.m_ballCounter = ballCounter;

        this.addRequirements(feeder, lowMagazine, highMagazine);

        SequentialCommandGroup increment = new SequentialCommandGroup();

        for (int i = 0; i < numFeeds; i++) {
            increment.addCommands(
                    new IncrementHighMagazine(m_highMagazine, Constants.Magazine.HIGH_INDEX_BY_ONE_POS),
                    new WaitCommand(Constants.Magazine.NORMAL_BALL_INCREMENT_WAIT)
            );
        }

        this.addCommands(
                new RunCommand(
                        () -> {
                            m_lowMagazine.setPWM(Constants.Magazine.LOW_MAGAZINE_FEED_PWM);
                        }
                ).withInterrupt(
                        () -> m_ballCounter.transferIRHasBall()
                ),
                new RunCommand(
                        () -> {
                            m_feeder.setPWM(Constants.Feeder.FEED_PWM);
                        }
                ),
                increment
        );
    }

    @Override
    public void end(boolean interrupted) {
        m_lowMagazine.setPWM(0);
        m_highMagazine.setPWM(0);
        m_feeder.setPWM(0);
    }
}
