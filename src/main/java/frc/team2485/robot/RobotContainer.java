/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team2485.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import frc.team2485.WarlordsLib.oi.Deadband;
import frc.team2485.WarlordsLib.oi.WL_XboxController;
import frc.team2485.robot.commands.IncrementHighMagazine;
import frc.team2485.robot.subsystems.Drivetrain;
import frc.team2485.robot.subsystems.HighMagazine;
import frc.team2485.robot.subsystems.LowMagazine;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.team2485.WarlordsLib.Limelight;
import frc.team2485.WarlordsLib.SmartDashboardHelper;
import frc.team2485.WarlordsLib.oi.Deadband;
import frc.team2485.WarlordsLib.oi.WL_XboxController;
import frc.team2485.robot.commands.shooter.SetHood;
import frc.team2485.robot.commands.shooter.Shoot;
import frc.team2485.robot.subsystems.Feeder;
import frc.team2485.robot.subsystems.Flywheels;
import frc.team2485.robot.subsystems.Hood;

import java.time.Instant;

public class RobotContainer {

    private WL_XboxController m_jack;
    private WL_XboxController m_suraj;

    private Drivetrain m_drivetrain;
    private LowMagazine m_lowMagazine;
    private HighMagazine m_highMagazine;
    private Feeder m_feeder;
    private Flywheels m_flywheels;
    private Hood m_hood;
    private Limelight m_limelight;


    SetHood setHood;

    private Command m_autoCommand;

    public RobotContainer() {

        m_drivetrain = new Drivetrain();
        m_lowMagazine = new LowMagazine();
        m_highMagazine = new HighMagazine(m_lowMagazine::getTransferIR);

        m_feeder = new Feeder();
        m_flywheels = new Flywheels();
        m_hood = new Hood(m_feeder.getHoodEncoder());
        m_limelight = new Limelight();
        m_limelight.setLedMode(Limelight.LedMode.OFF);


        m_jack = new WL_XboxController(Constants.OI.JACK_PORT);
        m_suraj = new WL_XboxController(Constants.OI.SURAJ_PORT);

        configureCommands();
    }

    private void configureCommands() {

        m_suraj.getJoystickAxisButton(XboxController.Axis.kLeftTrigger, Constants.OI.SURAJ_LTRIGGER_THRESHOLD).whenHeld(
                new Shoot(m_flywheels, m_hood, m_limelight, () -> {
                    return -m_suraj.getY(GenericHID.Hand.kRight);
                }));

        m_jack.getJoystickButton(XboxController.Button.kA).whenHeld(
                new RunCommand(() ->
                        m_feeder.setPWM(Deadband.linearScaledDeadband(
                                m_jack.getY(GenericHID.Hand.kRight), Constants.OI.XBOX_DEADBAND)))
        ).whenReleased(
                new RunCommand(() -> {
                    m_feeder.setPWM(0);
                })
        );


        m_jack.getJoystickButton(XboxController.Button.kB).whenHeld(
                new RunCommand(() -> {
                    m_hood.setPWM(Deadband.linearScaledDeadband(m_jack.getY(GenericHID.Hand.kRight), Constants.OI.XBOX_DEADBAND));
                })
        ).whenReleased(
                new RunCommand(() -> {
                    m_hood.setPWM(0);
                })
        );


//        m_highMagazine.setDefaultCommand(
//                new ConditionalCommand(
//                        new IncrementHighMagazine(m_highMagazine, Constants.Magazine.HIGH_INDEX_BY_ONE_POS),
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
                        new IncrementHighMagazine(m_highMagazine, Constants.Magazine.HIGH_INDEX_BY_ONE_POS),
                        new InstantCommand(() -> {
                            m_highMagazine.setPWM(0);
                        }),
                        () -> {
                            return m_highMagazine.getTransferIR() && m_highMagazine.getNumBalls() < Constants.Magazine.HIGH_MAGAZINE_BALL_CAPACITY;
//                            return m_highMagazine.g
//                            etTransferIR();
                        }
                )
        ).whenReleased(
                new InstantCommand(() -> {
                    m_highMagazine.setPWM(0);
                })
        );

        m_suraj.getJoystickButton(XboxController.Button.kBumperLeft).whileHeld(
                new ConditionalCommand(
                        new InstantCommand(() -> {
                            m_lowMagazine.setPWM(Constants.Magazine.LOW_BELT_PWM);
                        }),
                        new InstantCommand(() -> {
                            m_lowMagazine.setPWM(0);
                        }),
                        () -> {
                            return m_highMagazine.getNumBalls() < Constants.Magazine.HIGH_MAGAZINE_BALL_CAPACITY || !m_lowMagazine.getTransferIR();

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
//        m_suraj.getJoystickButton(XboxController.Button.kA).whileHeld(
//                new RunCommand(
//                        () -> {
//                            m_lowMagazine.setPWM(Constants.Magazine.FAST_INTAKE_PWM);
//                            m_highMagazine.setPWM(Constants.Magazine.FAST_INTAKE_PWM);
//                        }
//                )
//        ).whenReleased(
//                new InstantCommand(
//                        () -> {
//                            m_lowMagazine.setPWM(0);
//                            m_highMagazine.setPWM(0);
//                        }
//                )
//        );

        // Manual low magazine
//        m_suraj.getJoystickButton(XboxController.Button.kB).whileHeld(
//                new RunCommand(
//                        () -> {
//                            m_lowMagazine.setPWM(Deadband.linearScaledDeadband(m_suraj.getTriggerAxis(GenericHID.Hand.kRight), Constants.OI.XBOX_DEADBAND));
//
//                        }
//                )
//        );

//         Increment ball into shooter
        m_suraj.getJoystickButton(XboxController.Button.kBumperRight).whileHeld(
                new SequentialCommandGroup(
                        new InstantCommand(()->m_lowMagazine.setPWM(-0.5))
                                .alongWith(new IncrementHighMagazine(m_highMagazine, Constants.Magazine.HIGH_INDEX_BY_ONE_POS)),
                        new WaitCommand(Constants.Magazine.NORMAL_BALL_INCREMENT_TIMEOUT)
                )
        ).whenReleased(
                new InstantCommand(()->m_lowMagazine.setPWM(0))
        );

        //set both feeder and shooter to 0.2

        m_suraj.getJoystickButton(XboxController.Button.kY).whileHeld(
                new InstantCommand(()->m_feeder.setPWM(-0.3)).alongWith(
                        new InstantCommand(()->m_flywheels.setPWM(-0.3))
                )
        ).whenReleased(
                new InstantCommand(()->m_flywheels.setPWM(0)).alongWith(
                        new InstantCommand(()->m_feeder.setPWM(0))
                )
        );

        SmartDashboard.putData("reset encoder", new InstantCommand(() -> {
            m_highMagazine.resetEncoder(0);
        }));


        setHood = new SetHood(m_hood, 0);
    }

    public Command getAutonomousCommand() {
        // temporary!
        m_autoCommand = new RunCommand(() -> {
        });

        return m_autoCommand;
    }


    public void testInit() {

        SmartDashboard.putBoolean("Tune Enable", false);
        m_flywheels.tuneInit();
    }

    public void testPeriodic() {
        boolean enabled = SmartDashboard.getBoolean("Tune Enable", false);
        if (enabled) {
            m_flywheels.tunePeriodic();
            //m_feeder.tunePeriodic();
            m_lowMagazine.tunePeriodic();
        } else {
            m_lowMagazine.setPWM(-Deadband.linearScaledDeadband(m_suraj.getY(GenericHID.Hand.kLeft), Constants.OI.XBOX_DEADBAND));
            m_highMagazine.setPWM(-Deadband.linearScaledDeadband(m_suraj.getY(GenericHID.Hand.kRight), Constants.OI.XBOX_DEADBAND));
        }
    }


}
