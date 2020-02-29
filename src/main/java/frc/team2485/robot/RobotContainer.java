/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team2485.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Axis;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;

import frc.team2485.WarlordsLib.Limelight;
import frc.team2485.WarlordsLib.oi.Deadband;
import frc.team2485.WarlordsLib.oi.WL_XboxController;
import frc.team2485.robot.commands.IncrementHighMagazine;
import frc.team2485.robot.commands.paths.LineToRightTrenchPath;
import frc.team2485.robot.commands.paths.RightTrenchToLinePath;
import frc.team2485.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.team2485.robot.commands.SetHood;
import frc.team2485.robot.commands.Shoot;

import frc.team2485.robot.commands.TurretSetAngle;
import frc.team2485.robot.commands.IntakeArmMove;
import frc.team2485.robot.subsystems.Drivetrain;

public class RobotContainer {

    private WL_XboxController m_jack;
    private WL_XboxController m_suraj;

    private Drivetrain m_drivetrain;
    private LowMagazine m_lowMagazine;
    private HighMagazine m_highMagazine;
    private BallCounter m_ballCounter;
    private Feeder m_feeder;
    private Flywheels m_flywheels;
    private Hood m_hood;
    private Climber m_climber;
    private Turret m_turret;
    private IntakeArm m_intakeArm;



    private Command m_autoCommand;

    private SetHood setHood;

    public RobotContainer() {

        m_jack = new WL_XboxController(Constants.OI.JACK_PORT);
        m_suraj = new WL_XboxController(Constants.OI.SURAJ_PORT);

        m_drivetrain = new Drivetrain();
        m_lowMagazine = new LowMagazine();
        m_highMagazine = new HighMagazine();
        m_ballCounter = new BallCounter(m_lowMagazine::getEncoderVelocity, m_highMagazine::getEncoderVelocity);
        m_feeder = new Feeder();
        m_flywheels = new Flywheels();
        m_hood = new Hood(m_feeder.getHoodEncoder());
        m_turret = new Turret();
        m_intakeArm = new IntakeArm();
        m_climber = new Climber();

        this.configureCommands();
    }

    private void configureCommands() {

        this.configureDrivetrainCommands();
        this.configureIntakeArmCommands();
        this.configureClimberCommands();
        this.configureHoodCommands();
        this.configureTurretCommands();
        this.configureFlywheelsCommands();

        this.configureShootingCommands();
        this.configureIntakingCommands();

        // Toggle Limelight LED
        m_suraj.getJoystickButton(XboxController.Button.kBack).whenPressed(new InstantCommand(() -> {
            m_turret.getLimelight().toggleLed();
        }));

        m_jack.getJoystickButton(XboxController.Button.kBack).whenPressed(new InstantCommand(() -> {
            m_turret.getLimelight().toggleLed();
        }));
    }

    public void configureDrivetrainCommands() {
        m_drivetrain.setDefaultCommand(
                new RunCommand(() -> {
                    m_drivetrain.curvatureDrive(
                            Deadband.cubicScaledDeadband(
                                    m_jack.getTriggerAxis(GenericHID.Hand.kRight) - m_jack.getTriggerAxis(GenericHID.Hand.kLeft),
                                    Constants.OI.XBOX_DEADBAND),
                            Deadband.cubicScaledDeadband(
                                    ((m_jack.getX(GenericHID.Hand.kLeft) * m_jack.getX(GenericHID.Hand.kLeft)) * (m_jack.getX(GenericHID.Hand.kLeft) / Math.abs(m_jack.getX(GenericHID.Hand.kLeft)))),
                                    Constants.OI.XBOX_DEADBAND)
                                    * Constants.Drivetrain.STEERING_SCALE,
                            m_jack.getXButton());
                }, m_drivetrain)
        );
    }

    public void configureIntakeArmCommands() {
        m_intakeArm.setDefaultCommand(
                new IntakeArmMove(m_intakeArm, IntakeArmMove.IntakeArmPosition.TOP, Constants.IntakeArm.SPEED)
        );

        m_jack.getJoystickButton(XboxController.Button.kBumperRight)
                .whenHeld(new IntakeArmMove(m_intakeArm, IntakeArmMove.IntakeArmPosition.BOTTOM, Constants.IntakeArm.SPEED));
    }

    public void configureIntakingCommands() {

        m_jack.getJoystickButton(XboxController.Button.kBack).whenPressed(
                new InstantCommand(() -> {
                    m_ballCounter.setNumBallsLow(0);
                    m_ballCounter.setNumBallsHigh(0);
                })
        );

        // Increment high magazine
        m_jack.getJoystickButton(XboxController.Button.kA).whileHeld(
                new ConditionalCommand(
                        new IncrementHighMagazine(m_highMagazine, Constants.Magazine.HIGH_INDEX_BY_ONE_POS).withInterrupt(() -> !m_ballCounter.getTransferIR()),
                        new InstantCommand(() -> {
                            m_highMagazine.setPWM(0);
                        }),
                        () -> {
                            return m_ballCounter.getTransferIR()
                                    && (m_ballCounter.getNumBallsHigh() <= Constants.Magazine.HIGH_MAGAZINE_BALL_CAPACITY);
                        }
                )
        ).whenReleased(
                new InstantCommand(() -> {
                    m_highMagazine.setPWM(0);
                })
        );

        // Run low magazine
        m_jack.getJoystickButton(XboxController.Button.kA).whileHeld(
                new ConditionalCommand(
                        new InstantCommand(() -> {
                            m_lowMagazine.setPWM(Constants.Magazine.LOW_BELT_INTAKE_PWM);
//                            m_lowMagazine.setVelocity(-30);
                        }),
                        new InstantCommand(() -> {
                            m_lowMagazine.setPWM(0);
                        }),
                        () -> {
                            return !((m_ballCounter.getNumBallsHigh() >= Constants.Magazine.HIGH_MAGAZINE_BALL_CAPACITY
                                    || m_ballCounter.getNumBallsHigh() >= 3 && m_ballCounter.getExitIR())
                                    && m_ballCounter.getTransferIR());

                        }
                )
        ).whenReleased(
                new InstantCommand(
                        () ->
                                m_lowMagazine.setPWM(0)
                )
        );


        m_jack.getJoystickButton(XboxController.Button.kB).whileHeld(
                new RunCommand(() -> {
                    m_lowMagazine.setPWM(Constants.Magazine.OUTTAKE_PWM);
                    m_highMagazine.setPWM(Constants.Magazine.OUTTAKE_PWM);
                    m_feeder.setPWM(Constants.Feeder.OUTTAKE_PWM);
                })
        ).whenReleased(
                new InstantCommand(() -> {
                    m_lowMagazine.setPWM(0);
                    m_highMagazine.setPWM(0);
                    m_feeder.setPWM(0);
                })
        );
    }

    public void configureShootingCommands() {

        // Increment ball into shooter
        m_suraj.getJoystickButton(XboxController.Button.kBumperRight).whileHeld(
                new SequentialCommandGroup(
                        new InstantCommand(() -> {
                            m_lowMagazine.setPWM(-0.5);
                            m_feeder.setPWM(-0.8);
                        }).alongWith(
                                new IncrementHighMagazine(m_highMagazine, Constants.Magazine.HIGH_INDEX_BY_ONE_POS)),
                        new WaitCommand(Constants.Magazine.NORMAL_BALL_INCREMENT_TIMEOUT)
                )
        ).whenReleased(
                new InstantCommand(() -> {
                    m_lowMagazine.setPWM(0);
                    m_highMagazine.setPWM(0);
                    m_feeder.setPWM(0);
                }
                )
        );

        // Spin up and aim
        m_suraj.getJoystickButton(XboxController.Button.kBumperLeft).whileHeld(
                new ParallelCommandGroup(
                        new Shoot(m_flywheels, m_hood, m_turret.getLimelight(),
                                () -> -Deadband.cubicScaledDeadband(
                                        m_suraj.getY(GenericHID.Hand.kLeft),
                                        Constants.OI.XBOX_DEADBAND),
                                () -> Constants.Flywheels.RPM_ADJUST * (Deadband.cubicScaledDeadband(
                                        m_suraj.getTriggerAxis(GenericHID.Hand.kRight),
                                        Constants.OI.XBOX_DEADBAND)
                                        - Deadband.cubicScaledDeadband(
                                        m_suraj.getTriggerAxis(GenericHID.Hand.kLeft),
                                        Constants.OI.XBOX_DEADBAND)),
                                () -> Constants.Hood.HOOD_MANUAL_ADJUST * -Deadband.cubicScaledDeadband(
                                        m_suraj.getX(GenericHID.Hand.kRight),
                                        Constants.OI.XBOX_DEADBAND)
                        ),
                        // Turret Seek
                        new ConditionalCommand(
                                new TurretSetAngle(m_turret, () -> {
                                    return m_turret.getEncoderPosition()
                                            + m_turret.getLimelight().getTargetHorizontalOffset(0)
                                            + Deadband.cubicScaledDeadband(
                                            m_suraj.getX(GenericHID.Hand.kRight),
                                            Constants.OI.XBOX_DEADBAND);
                                }),
                                new SequentialCommandGroup(
                                        new TurretSetAngle(m_turret, Constants.Turret.MIN_POSITION + Constants.Turret.BUFFER_ZONE_SIZE, true),
                                        new TurretSetAngle(m_turret, Constants.Turret.MAX_POSITION - Constants.Turret.BUFFER_ZONE_SIZE, true)
                                ).withInterrupt(() -> m_turret.getLimelight().hasValidTarget()),
                                () -> m_turret.getLimelight().hasValidTarget())
                )
        ).whenReleased(new InstantCommand(() -> {
            m_flywheels.setPWM(0);
            m_hood.setPWM(0);
        }));
    }

    public void configureTurretCommands() {
        // Turret manual control
        m_turret.setDefaultCommand(
                new RunCommand(() ->
                        m_turret.setClampedPWM(
                                0.5 * Deadband.cubicScaledDeadband(
                                        m_suraj.getX(GenericHID.Hand.kLeft),
                                        Constants.OI.XBOX_DEADBAND)
                        )
                        , m_turret)
        );

        m_turret.setDefaultCommand(
                new TurretSetAngle(m_turret,
                        () -> {
                            double setpoint = m_turret.getEncoderPosition();
                            double pwm = getAxis(m_suraj, Axis.kLeftX);

                            if (pwm > 0) {
                                setpoint += (m_turret.getMaxAngle() - m_turret.getEncoderPosition()) * pwm;
                            } else if (pwm < 0) {

                            }

                            return setpoint;
                        }
                )
        );

        // Limelight align
//        m_suraj.getJoystickButton(XboxController.Button.kBumperLeft).whenHeld(
//                new TurretSetAngle(m_turret, () -> {
//                    return m_turret.getEncoderPosition() + m_turret.getLimelight().getTargetHorizontalOffset(0);
//                })
////        );


        // Field Centric Control
//        m_suraj.getJoystickButton(XboxController.Button.kX).whenHeld(
//                new TurretFieldCentricAdjust(m_turret,
//                        () -> {
//                            return Constants.Turret.TURRET_SPEED * Deadband.cubicScaledDeadband(
//                                    m_suraj.getX(GenericHID.Hand.kLeft),
//                                    Constants.OI.XBOX_DEADBAND);
//                        },
//                        () -> {
//
//
//                            SmartDashboard.putNumber("Pigeon Heading", -pigeon.getFusedHeading());
//                            return -pigeon.getFusedHeading();
//                        }
//                )
//        );
    }

    public void configureHoodCommands() {
        m_hood.setDefaultCommand(new RunCommand(
                () -> {
                    m_hood.setPWM(-Deadband.linearScaledDeadband(m_suraj.getY(GenericHID.Hand.kRight), Constants.OI.XBOX_DEADBAND));
                }, m_hood
        ));

    }

    public void configureFlywheelsCommands() {
//        m_flywheels.setPWM(-Deadband.linearScaledDeadband(m_suraj.getTriggerAxis(GenericHID.Hand.kLeft), Constants.OI.XBOX_DEADBAND));

    }

    public void configureClimberCommands() {
        m_jack.getJoystickButton(XboxController.Button.kY).whileHeld(
                new RunCommand(() -> {
                    m_climber.setPWM(Deadband.linearScaledDeadband(m_jack.getY(GenericHID.Hand.kRight), Constants.OI.XBOX_DEADBAND));
                })
        );
    }

    public Command getAutonomousCommand() {
        // temporary!

        m_autoCommand = new RunCommand(() -> {

        });

        return m_autoCommand;
    }

    public void testInit() {
        if (Constants.TUNE_MODE) {
            SmartDashboard.putBoolean(Constants.TUNE_ENABLE_LABEL, false);
        }

        SmartDashboard.putBoolean(Constants.Turret.ZERO_TURRET_LABEL, false);
        SmartDashboard.putBoolean(Constants.Drivetrain.RESET_GYRO_LABEL, false);
    }

    public void testPeriodic() {
        if (SmartDashboard.getBoolean(Constants.Turret.ZERO_TURRET_LABEL, false)) {
            m_turret.resetEncoderPosition(0);
            SmartDashboard.putBoolean(Constants.Turret.ZERO_TURRET_LABEL, false);
        }

        if (SmartDashboard.getBoolean(Constants.Drivetrain.RESET_GYRO_LABEL, false)) {
            m_drivetrain.setHeading(0);
            SmartDashboard.putBoolean(Constants.Drivetrain.RESET_GYRO_LABEL, false);
        }


        if (Constants.TUNE_MODE) {
            boolean enable = SmartDashboard.getBoolean(Constants.TUNE_ENABLE_LABEL, false);
            if (enable) {
//                m_lowMagazine.tunePeriodic();
                m_highMagazine.tunePeriodic();
//                m_turret.tunePeriodic();
            } else {
//                m_lowMagazine.setPWM(-getAxis(m_jack, Axis.kRightY));
//                m_highMagazine.setPWM(-getAxis(m_jack, Axis.kLeftY));
                m_turret.setPWM(getAxis(m_jack, Axis.kLeftX));
//                m_feeder.setPWM(Deadband.linearScaledDeadband(m_suraj.getY(GenericHID.Hand.kLeft), Constants.OI.XBOX_DEADBAND));
//                m_flywheels.setPWM(-Deadband.linearScaledDeadband(m_suraj.getY(GenericHID.Hand.kRight), Constants.OI.XBOX_DEADBAND));
//                m_hood.setPWM(-Deadband.linearScaledDeadband(m_jack.getY(GenericHID.Hand.kLeft), Constants.OI.XBOX_DEADBAND));
            }
        }
    }

    private double getAxis(WL_XboxController controller, Axis axis) {
        return Deadband.linearScaledDeadband(controller.getRawAxis(axis.value), Constants.OI.XBOX_DEADBAND);
    }
}

