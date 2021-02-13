// package frc.team2485.robot.commands;

// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.*;
// import frc.team2485.robot.Constants;
// import frc.team2485.robot.subsystems.BallCounter;
// import frc.team2485.robot.subsystems.HighMagazine;
// import frc.team2485.robot.subsystems.LowMagazine;

// public class IntakeBalls extends ParallelCommandGroup {

    // private LowMagazine m_lowMagazine;
    // private HighMagazine m_highMagazine;
    // private BallCounter m_ballCounter;

    // private IncrementHighMagazine m_incrementHighMagazine;

    // public IntakeBalls(LowMagazine lowMagazine, HighMagazine highMagazine, BallCounter ballCounter) {
    //     m_lowMagazine = lowMagazine;
    //     m_highMagazine = highMagazine;
    //     m_ballCounter = ballCounter;

    //     addRequirements(lowMagazine, highMagazine);

    //     m_incrementHighMagazine = new IncrementHighMagazine(highMagazine, Constants.Magazine.HIGH_INDEX_BY_ONE_POS);

    //     this.addCommands(
    //             new RunCommand(
    //                     () -> {
    //                         if (!((m_ballCounter.getNumBallsHigh() >= Constants.Magazine.HIGH_MAGAZINE_BALL_CAPACITY
    //                                 || (m_ballCounter.getNumBallsHigh() >= 3 && m_ballCounter.exitIRHasBall()))
    //                                 && m_ballCounter.transferIRHasBall())) {
    //                             m_lowMagazine.runVelocityPID(SmartDashboard.getNumber("Intake Speed", -40));
    //                         } else {
    //                             m_lowMagazine.setPWM(0);
    //                         }
    //                     }
    //             ),
    //             new RunCommand(
    //                     () -> {
    //                         if (m_ballCounter.transferIRHasBall() &&
    //                                 (m_ballCounter.getNumBallsHigh() < Constants.Magazine.HIGH_MAGAZINE_BALL_CAPACITY)
    //                                 && m_incrementHighMagazine.isFinished()) {
    //                             System.out.println("Scheduled");
    //                             m_incrementHighMagazine.schedule();
    //                         }
    //                     }
    //             )
    //     );
    // }

    // @Override
    // public void initialize() {

    //     m_lowMagazine.resetPIDs();
    //     m_highMagazine.resetPIDs();
    // }

    // @Override
    // public void end(boolean interrupted) {
    //     m_lowMagazine.setPWM(0);
    //     m_highMagazine.setPWM(0);
    // }
// }
