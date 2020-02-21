/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team2485.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import frc.team2485.WarlordsLib.SmartDashboardHelper;
import frc.team2485.WarlordsLib.oi.Deadband;
import frc.team2485.WarlordsLib.oi.WL_XboxController;
import frc.team2485.robot.commands.IncrementMagazine;
import frc.team2485.robot.subsystems.Drivetrain;
import frc.team2485.robot.subsystems.HighMagazine;
import frc.team2485.robot.subsystems.LowMagazine;

public class RobotContainer {

    private WL_XboxController m_jack;
    private WL_XboxController m_suraj;

    private Drivetrain m_drivetrain;
    private LowMagazine m_lowMagazine;
    private HighMagazine m_highMagazine;

    private Command m_autoCommand;

    public RobotContainer() {

        m_drivetrain = new Drivetrain();
        m_lowMagazine = new LowMagazine();
        m_highMagazine = new HighMagazine(m_lowMagazine::getTransferIR);

        m_jack = new WL_XboxController(Constants.OI.JACK_PORT);
        m_suraj = new WL_XboxController(Constants.OI.SURAJ_PORT);

        configureCommands();
    }

    private void configureCommands() {
        m_drivetrain.setDefaultCommand(new RunCommand(() -> {
                    m_drivetrain.curvatureDrive(
                            Deadband.cubicScaledDeadband(
                                    m_jack.getTriggerAxis(GenericHID.Hand.kRight)
                                            - m_jack.getTriggerAxis(GenericHID.Hand.kLeft),
                                    Constants.OI.XBOX_DEADBAND),
                            Deadband.cubicScaledDeadband(
                                    m_jack.getX(GenericHID.Hand.kLeft),
                                    Constants.OI.XBOX_DEADBAND),
                            m_jack.getXButton());
                }, m_drivetrain)
        );

//        m_highMagazine.setDefaultCommand(
//                new ConditionalCommand(
//                        new IncrementMagazine(m_highMagazine, Constants.Magazine.HIGH_INDEX_BY_ONE_POS),
//                        new InstantCommand(() -> {
//                            m_highMagazine.setPWM(0);
//                        }),
//                        () -> {
//                            return m_highMagazine.getTransferIR() && m_highMagazine.getNumBalls() <= Constants.Magazine.HIGH_MAGAZINE_BALL_CAPACITY;
//                        }
//                )
//        );

        m_suraj.getJoystickButton(XboxController.Button.kBumperLeft).whileHeld(
                new ConditionalCommand(
                        new IncrementMagazine(m_highMagazine, Constants.Magazine.HIGH_INDEX_BY_ONE_POS),
                        new RunCommand(() -> {
                            m_highMagazine.setPWM(0);
                        }),
                        () -> {
//                            return m_highMagazine.getTransferIR() && m_highMagazine.getNumBalls() <= Constants.Magazine.HIGH_MAGAZINE_BALL_CAPACITY;
                            return m_highMagazine.getTransferIR();
                        }
                )
        ).whenReleased(
                new InstantCommand(()-> {
                    m_highMagazine.setPWM(0);
                })
        );

        m_suraj.getJoystickButton(XboxController.Button.kBumperRight).whenHeld(
                new ConditionalCommand(
                        new RunCommand(() -> {
                            m_lowMagazine.setPWM(Constants.Magazine.LOW_BELT_PWM);
                        }),
                        new InstantCommand(() -> {
                            m_lowMagazine.setPWM(0);
                        }),
                        () -> {
                            return m_highMagazine.getNumBalls() <= Constants.Magazine.HIGH_MAGAZINE_BALL_CAPACITY && !m_lowMagazine.getTransferIR();
                        }
                )
        ).whenReleased(
                new InstantCommand(
                        () -> {
                            m_lowMagazine.setPWM(0);
                        }
                )
        );

        // Feed to shooter
        m_suraj.getJoystickButton(XboxController.Button.kA).whileHeld(
                new RunCommand(
                        () -> {
                            m_lowMagazine.setPWM(Constants.Magazine.FAST_INTAKE_PWM);
                            m_highMagazine.setPWM(Constants.Magazine.FAST_INTAKE_PWM);
                        }
                )
        ).whenReleased(
                new InstantCommand(
                        () -> {
                            m_lowMagazine.setPWM(0);
                            m_highMagazine.setPWM(0);
                        }
                )
        );

        // Manual low magazine
        m_suraj.getJoystickButton(XboxController.Button.kB).whileHeld(
                new RunCommand(
                        () -> {
                            m_lowMagazine.setPWM(Deadband.linearScaledDeadband(m_suraj.getTriggerAxis(GenericHID.Hand.kRight), Constants.OI.XBOX_DEADBAND));

                        }
                )
        );

        // Increment ball into shooter
        m_suraj.getJoystickButton(XboxController.Button.kBumperRight).whileHeld(
                new SequentialCommandGroup(
                        new IncrementMagazine(m_lowMagazine, Constants.Magazine.LOW_INTAKE_BY_ONE_POS)
                                .alongWith(new IncrementMagazine(m_lowMagazine, Constants.Magazine.LOW_INTAKE_BY_ONE_POS)),
                        new WaitCommand(Constants.Magazine.NORMAL_BALL_INCREMENT_TIMEOUT)
                )
        );

        SmartDashboard.putData("reset encoder", new InstantCommand(() -> {
            m_highMagazine.resetEncoder(0);
        }));

    }

    public void resetAll() {
        m_drivetrain.resetEncoders(0, 0);
    }

    public Command getAutonomousCommand() {
        // temporary!
        m_autoCommand = new RunCommand(() -> {
        });

        return m_autoCommand;
    }

    public void tunePeriodic(boolean enable) {
//        m_lowMagazine.tunePeriodic(enable);
        if (enable) {
            m_highMagazine.tunePeriodic();
        } else {
//            m_lowMagazine.setPWM(-Deadband.linearScaledDeadband(m_jack.getY(GenericHID.Hand.kLeft), Constants.OI.XBOX_DEADBAND));
            m_highMagazine.setPWM(-Deadband.linearScaledDeadband(m_jack.getY(GenericHID.Hand.kRight), Constants.OI.XBOX_DEADBAND));

            //            m_lowMagazine.setPWM(0);
        }

        SmartDashboard.putNumber("Low Magazine Position", m_lowMagazine.getEncoderPosition());
    }
}
