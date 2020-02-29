package frc.team2485.robot;

import edu.wpi.first.wpilibj.Sendable;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.*;
import frc.team2485.robot.commands.IncrementHighMagazine;
import frc.team2485.robot.commands.IntakeArmMove;
import frc.team2485.robot.commands.Shoot;
import frc.team2485.robot.commands.TurretSetAngle;
import frc.team2485.robot.commands.paths.LineToRightTrenchPath;
import frc.team2485.robot.commands.paths.RightTrenchToLinePath;
import frc.team2485.robot.subsystems.*;

public class AutoChooser  {

  private Command m_leftTrenchAuto;
  private SendableChooser<Command> m_dashboardChooser = new SendableChooser<>();


    public AutoChooser(Drivetrain drivetrain, Feeder feeder, Flywheels flywheels, Hood hood, Turret turret, LowMagazine lowMagazine, HighMagazine highMagazine, IntakeArm intakeArm, BallCounter ballCounter) {
        //GENERAL AUTONOMOUS COMMANDS

        //SHOOT WITH FLYWHEELS AND HOOD
        Command shoot = new Shoot(flywheels,
                hood,
                turret.getLimelight(),
                ()->Constants.Autonomous.PORT_ENTRANCE_Y_VELOCITY,
                ()->Constants.Autonomous.RPM_ADJUST,
                ()->Constants.Autonomous.HOOD_ADJUST);

        //FEED TO SHOOTER WITH MAGAZINE AND FEEDER WHEELS
        Command indexToShooter = new SequentialCommandGroup(
                new InstantCommand(() -> {
                    lowMagazine.setPWM(-0.5);
                    feeder.setPWM(-0.8);
                }).alongWith(
                        new IncrementHighMagazine(highMagazine, Constants.Magazine.HIGH_INDEX_BY_ONE_POS)),
                new WaitCommand(Constants.Magazine.NORMAL_BALL_INCREMENT_TIMEOUT)
        );

        //ALIGN TURRET TO GOAL USING LIMELIGHT
        Command alignTurret = new TurretSetAngle(turret, () -> {
                    return turret.getEncoderPosition() + turret.getLimelight().getTargetHorizontalOffset(0);
                });

        //SHOOT WITH WHEELS, FEEDER, HOOD, TURRET, AND MAGAZINE
        Command shootWithTurretAndMagazine = new ParallelCommandGroup(
                shoot,
                new SequentialCommandGroup(new WaitCommand(Constants.Autonomous.SHOOTING_WARMUP_DELAY), indexToShooter),
                alignTurret
        );

        //MAGAZINE INTAKE (HIGH)
        Command highMagazineIntake = new ConditionalCommand(
                new IncrementHighMagazine(highMagazine, Constants.Magazine.HIGH_INDEX_BY_ONE_POS).withInterrupt(() -> !(ballCounter.getTransferIR())),
                new InstantCommand(() -> {
                    highMagazine.setPWM(0);
                }),
                () -> {
                    return ballCounter.getTransferIR()
                            && (ballCounter.getNumBallsHigh() <= Constants.Magazine.HIGH_MAGAZINE_BALL_CAPACITY);
                });

        //MAGAZINE INTAKE(LOW)
        Command lowMagazineIntake = new ConditionalCommand(
                new InstantCommand(() -> {
                    lowMagazine.setPWM(Constants.Magazine.LOW_BELT_INTAKE_PWM);
//                            m_lowMagazine.setVelocity(-30);
                }),
                new InstantCommand(() -> {
                    lowMagazine.setPWM(0);
                }),
                () -> {
                    return !((ballCounter.getNumBallsHigh() >= Constants.Magazine.HIGH_MAGAZINE_BALL_CAPACITY
                            || ballCounter.getNumBallsHigh() >= 3 && ballCounter.getExitIR())
                            && ballCounter.getTransferIR());

                }
        );

        //MAGAZINE INTAKE
        Command magazineIntake = new ParallelCommandGroup(highMagazineIntake, lowMagazineIntake);

        //MAGAZINE AND ARM INTAKE (always on)
        Command ballIntake = new ParallelCommandGroup(magazineIntake, new IntakeArmMove(intakeArm, IntakeArmMove.IntakeArmPosition.BOTTOM, Constants.IntakeArm.SPEED));

        m_leftTrenchAuto = new ParallelCommandGroup(
                ballIntake,
                new SequentialCommandGroup(
                       shootWithTurretAndMagazine,
                        new LineToRightTrenchPath(drivetrain, turret.getLimelight()),
                        new RightTrenchToLinePath(drivetrain, turret.getLimelight()),
                        shootWithTurretAndMagazine
                )
        );
        m_dashboardChooser.addOption("Left Trench", m_leftTrenchAuto);


        this.addToShuffleboard();
    }



    private void addToShuffleboard() {
        ShuffleboardTab tab = Shuffleboard.getTab("Autonomous");
        tab.add(m_dashboardChooser);
    }

    public Command getAutonomousCommand() {
        return m_dashboardChooser.getSelected();
    }


}
