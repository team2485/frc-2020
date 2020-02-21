/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team2485.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.team2485.WarlordsLib.Limelight;
import frc.team2485.WarlordsLib.oi.Deadband;
import frc.team2485.WarlordsLib.oi.WL_XboxController;
import frc.team2485.robot.commands.IntakeArmMove;
import frc.team2485.robot.subsystems.Drivetrain;
import frc.team2485.robot.subsystems.IntakeArm;
import frc.team2485.robot.commands.shooter.SetHood;
import frc.team2485.robot.commands.shooter.Shoot;
import frc.team2485.robot.subsystems.Feeder;
import frc.team2485.robot.subsystems.Flywheels;
import frc.team2485.robot.subsystems.Hood;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import frc.team2485.WarlordsLib.oi.Deadband;
import frc.team2485.WarlordsLib.oi.WL_XboxController;
import frc.team2485.robot.commands.IncrementMagazine;
import frc.team2485.robot.subsystems.Drivetrain;
import frc.team2485.robot.subsystems.HighMagazine;
import frc.team2485.robot.subsystems.LowMagazine;
import com.ctre.phoenix.sensors.PigeonIMU;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import frc.team2485.WarlordsLib.SmartDashboardHelper;
import frc.team2485.WarlordsLib.Tunable;
import frc.team2485.WarlordsLib.oi.Deadband;
import frc.team2485.WarlordsLib.oi.WL_XboxController;
import frc.team2485.robot.commands.TurretFieldCentricAdjust;
import frc.team2485.robot.commands.TurretSetAngle;
import frc.team2485.robot.subsystems.Turret;

import java.time.Instant;

public class RobotContainer {

    private WL_XboxController m_jack;
    private WL_XboxController m_suraj;

    private Feeder m_feeder;
    private Flywheels m_flywheels;
    private Hood m_hood;
    private Limelight m_limelight;

    private IntakeArm m_intakeArm;

    private Drivetrain m_drivetrain;
    private LowMagazine m_lowMagazine;
    private HighMagazine m_highMagazine;
    //    private Drivetrain m_drivetrain;
    private Turret m_turret;


    private Command m_autoCommand;


    public RobotContainer() {

        m_drivetrain = new Drivetrain();
        m_intakeArm = new IntakeArm();
        m_feeder = new Feeder();
        m_flywheels = new Flywheels();
        m_hood = new Hood(m_feeder.getHoodEncoder());
        m_lowMagazine = new LowMagazine();
        m_highMagazine = new HighMagazine(m_lowMagazine::getTransferIR);

        m_jack = new WL_XboxController(Constants.OI.JACK_PORT);
        m_suraj = new WL_XboxController(Constants.OI.SURAJ_PORT);
//        m_drivetrain = new Drivetrain();
        m_turret = new Turret();

        m_jack = new WL_XboxController(Constants.OI.JACK_PORT);
        m_suraj = new WL_XboxController(Constants.OI.SURAJ_PORT);

        configureCommands();
    }

    private void configureCommands() {
        /*
        DRIVETRAIN
         */

        m_drivetrain.setDefaultCommand(new RunCommand(() -> {
                    m_drivetrain.curvatureDrive(
                            Deadband.cubicScaledDeadband(
                                    m_jack.getTriggerAxis(GenericHID.Hand.kRight) - m_jack.getTriggerAxis(GenericHID.Hand.kLeft),
                                    Constants.OI.XBOX_DEADBAND
                            ),
                            Deadband.cubicScaledDeadband(
                                    m_jack.getX(GenericHID.Hand.kLeft),
                                    Constants.OI.XBOX_DEADBAND
                            ),
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

//        m_suraj.getJoystickButton(XboxController.Button.kBumperLeft).whileHeld(
//                new ConditionalCommand(
//                        new IncrementMagazine(m_highMagazine, Constants.Magazine.HIGH_INDEX_BY_ONE_POS),
//                        new InstantCommand(() -> {
//                            m_highMagazine.setPWM(0);
//                        }),
//                        () -> {
////                            return m_highMagazine.getTransferIR() && m_highMagazine.getNumBalls() <= Constants.Magazine.HIGH_MAGAZINE_BALL_CAPACITY;
//                            return m_highMagazine.getTransferIR();
//                        }
//                )
//        ).whenReleased(
//                new InstantCommand(()-> {
//                    m_highMagazine.setPWM(0);
//                })
//        );

        // Feed to shooter
//        m_suraj.getJoystickButton(XboxController.Button.kA).whileHeld(
//                new RunCommand(
//                        () -> {
//                            m_lowMagazine.setPWM(Constants.Magazine.FAST_INTAKE_PWM);
//                            m_highMagazine.setPWM(Constants.Magazine.FAST_INTAKE_PWM);
//                            m_feeder.setPWM(-0.5);
//                        }
//                )
//        ).whenReleased(
//                new InstantCommand(
//                        () -> {
//                            m_lowMagazine.setPWM(0);
//                            m_highMagazine.setPWM(0);
//                            m_feeder.setPWM(0);
//                        }
//                )
//        );

        // Manual low magazine
        m_suraj.getJoystickButton(XboxController.Button.kA).whileHeld(
                new RunCommand(
                        () -> {
                            m_lowMagazine.setPWM(-Deadband.linearScaledDeadband(m_suraj.getTriggerAxis(GenericHID.Hand.kLeft), Constants.OI.XBOX_DEADBAND));

                        }
                )
        ).whenReleased(
                new InstantCommand(
                        () -> {
                            m_lowMagazine.setPWM(0);
                        }
                )
        );

//        // Increment ball into shooter
//        m_suraj.getJoystickButton(XboxController.Button.kBumperRight).whileHeld(
//                new SequentialCommandGroup(
//                        new IncrementMagazine(m_lowMagazine, Constants.Magazine.LOW_INTAKE_BY_ONE_POS)
//                                .alongWith(new IncrementMagazine(m_lowMagazine, Constants.Magazine.LOW_INTAKE_BY_ONE_POS)),
//                        new WaitCommand(Constants.Magazine.NORMAL_BALL_INCREMENT_TIMEOUT)
//                )
//        );

        /*
        INTAKE ARM
         */

        m_jack.getJoystickButton(XboxController.Button.kA).whenHeld(new IntakeArmMove(m_intakeArm, IntakeArmMove.IntakeArmPosition.BOTTOM, Constants.IntakeArm.SPEED));
        m_jack.getJoystickButton(XboxController.Button.kB).whenHeld(new IntakeArmMove(m_intakeArm, IntakeArmMove.IntakeArmPosition.TOP, Constants.IntakeArm.SPEED));

        m_jack.getJoystickButton(XboxController.Button.kBumperLeft).whenHeld(
                new RunCommand(()-> m_intakeArm.setPWM(Deadband.linearScaledDeadband(m_jack.getY(GenericHID.Hand.kRight), 0.1)))
        ).whenReleased(
                new RunCommand(() -> m_intakeArm.setPWM(0))
        );


        m_hood.setDefaultCommand(new RunCommand(() -> {
            m_hood.setPWM(Deadband.linearScaledDeadband(m_suraj.getY(GenericHID.Hand.kRight), Constants.OI.XBOX_DEADBAND));
        }, m_hood));

        m_suraj.getJoystickButton(XboxController.Button.kBumperRight).whenPressed(
                new RunCommand(() -> {
                    m_flywheels.setPWM(-Deadband.linearScaledDeadband(m_suraj.getTriggerAxis(GenericHID.Hand.kRight), Constants.OI.XBOX_DEADBAND));
                }).alongWith(
                        new RunCommand(
                                () -> {
                                    double pwm = -Deadband.linearScaledDeadband(m_suraj.getTriggerAxis(GenericHID.Hand.kLeft), Constants.OI.XBOX_DEADBAND);
                                    m_lowMagazine.setPWM(pwm);
                                    m_highMagazine.setPWM(pwm);
                                    m_feeder.setPWM(pwm);
                                }
                        )
                )
        ).whenReleased(
                new InstantCommand(() -> {
                    m_flywheels.setPWM(0);
                    m_lowMagazine.setPWM(0);
                    m_highMagazine.setPWM(0);
                    m_feeder.setPWM(0);
                })
        );

        m_suraj.getJoystickButton(XboxController.Button.kBumperLeft).whileHeld(
                new RunCommand(() -> {
                    m_flywheels.setPWM(0.1 * Deadband.linearScaledDeadband(m_suraj.getTriggerAxis(GenericHID.Hand.kRight), Constants.OI.XBOX_DEADBAND));
                }).alongWith(
                        new RunCommand(
                                () -> {
                                    double pwm = Deadband.linearScaledDeadband(m_suraj.getTriggerAxis(GenericHID.Hand.kLeft), Constants.OI.XBOX_DEADBAND);
                                    m_lowMagazine.setPWM(pwm);
                                    m_highMagazine.setPWM(pwm);
                                    m_feeder.setPWM(pwm);
                                }
                        )
                )
        ).whenReleased(
                new InstantCommand(() -> {
                    m_flywheels.setPWM(0);
                    m_lowMagazine.setPWM(0);
                    m_highMagazine.setPWM(0);
                    m_feeder.setPWM(0);
                })
        );

        m_suraj.getJoystickButton(XboxController.Button.kB).whileHeld(new RunCommand(
                () -> {
//                    double pwm = -Deadband.linearScaledDeadband(m_suraj.getTriggerAxis(GenericHID.Hand.kLeft), Constants.OI.XBOX_DEADBAND);
//                    m_lowMagazine.setPWM(pwm);
                    m_highMagazine.setPWM(-0.5);
                    m_feeder.setPWM(-0.5);
                }
        )).whenReleased(
                new InstantCommand(() -> {

                    m_highMagazine.setPWM(0);
                    m_feeder.setPWM(0);
                })
        );

        // Turret manual control
        m_turret.setDefaultCommand(
                new RunCommand(() ->
                        m_turret.setPWM(
                                Deadband.cubicScaledDeadband(
                                        m_suraj.getX(GenericHID.Hand.kLeft),
                                        Constants.OI.XBOX_DEADBAND)
                        )
                        , m_turret)
        );

        // Limelight align
        m_suraj.getJoystickButton(XboxController.Button.kY).whenHeld(
                new TurretSetAngle(m_turret, () -> {
                    return m_turret.getEncoderPosition() + m_turret.getLimelight().getTargetHorizontalOffset(0);
                })
        );

        // Field Centric Control
        m_suraj.getJoystickButton(XboxController.Button.kX).whenHeld(
                new TurretFieldCentricAdjust(m_turret,
                        () -> {
                            return Constants.Turret.TURRET_SPEED * Deadband.cubicScaledDeadband(
                                    m_suraj.getX(GenericHID.Hand.kLeft),
                                    Constants.OI.XBOX_DEADBAND);
                        },
                        () -> {
                            return -m_drivetrain.getHeading();
                        }
                )
        );

        m_suraj.getJoystickButton(XboxController.Button.kBack).whenPressed(new InstantCommand(() -> {
            m_turret.getLimelight().toggleLed();
        }));
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

        boolean enable = SmartDashboard.getBoolean("Tune Enable", false);

        if (SmartDashboard.getBoolean("Zero Turret", false)) {
            m_turret.resetEncoderPosition(0);
            SmartDashboard.putBoolean("Zero Turret", false);
        }

//        if (SmartDashboard.getBoolean("Zero Gyro", false)) {
//            pigeon.setFusedHeading(0);
//            SmartDashboard.putBoolean("Zero Gyro", false);
//        }

        if (enable) {
            m_turret.tunePeriodic();
        } else {
            m_turret.setUnclampedPWM(-Deadband.linearScaledDeadband(m_jack.getY(GenericHID.Hand.kLeft), Constants.OI.XBOX_DEADBAND));
        }
    }
}
