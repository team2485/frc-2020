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
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;

import frc.team2485.WarlordsLib.Tunable;
import frc.team2485.WarlordsLib.oi.Deadband;
import frc.team2485.WarlordsLib.oi.WL_XboxController;
import frc.team2485.robot.CommandGroups.FlywheelsCommands;
import frc.team2485.robot.commands.*;
import frc.team2485.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;

import frc.team2485.robot.subsystems.Drivetrain;

import java.util.function.DoubleSupplier;

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


    private SendableChooser<Tunable> m_tuneChooser;

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
        m_intakeArm = new IntakeArm(m_drivetrain.getIntakeArmEncoder());
        m_climber = new Climber();

        m_tuneChooser = new SendableChooser<Tunable>();

        this.configureCommands();
    }

    public void gameInit() {
        m_turret.resetPIDs();
        m_hood.resetPID();
        m_flywheels.resetPIDs();
        m_highMagazine.resetPIDs();
        m_lowMagazine.resetPIDs();
        m_intakeArm.resetPIDs();
        m_feeder.resetPIDs();
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

        m_jack.getJoystickButton(XboxController.Button.kStart).whenPressed(new InstantCommand(() -> {
            m_ballCounter.setNumBallsHigh(0);
            m_ballCounter.setNumBallsLow(0);
        }));

        this.configureTuning();
    }

    public void configureDrivetrainCommands() {
        m_drivetrain.setDefaultCommand(
                new RunCommand(() -> {
                    m_drivetrain.curvatureDrive(
                            -Deadband.cubicScaledDeadband(
                                    Constants.Drivetrain.THROTTLE_SCALE * (m_jack.getTriggerAxis(GenericHID.Hand.kRight) - m_jack.getTriggerAxis(GenericHID.Hand.kLeft)),
                                    Constants.OI.XBOX_DEADBAND),
                            Deadband.cubicScaledDeadband(
                                    ((m_jack.getX(GenericHID.Hand.kLeft) * Math.abs(m_jack.getX(GenericHID.Hand.kLeft)))),
                                    Constants.OI.XBOX_DEADBAND)
                                    * Constants.Drivetrain.STEERING_SCALE,
                            m_jack.getXButton());
                }, m_drivetrain)
        );
    }

    public void configureIntakeArmCommands() {

//        m_intakeArm.setDefaultCommand(
//                new RunCommand(() -> {
//                    if (!m_intakeArm.getTopLimitSwitch()) {
//                        m_intakeArm.setPWM(Constants.IntakeArm.SPEED);
//                    } else {
//                        m_intakeArm.setPWM(0);
//                    }
//                }, m_intakeArm)
////                new ConditionalCommand(
////                                new IntakeArmMove(m_intakeArm, IntakeArmMove.IntakeArmPosition.TOP, Constants.IntakeArm.SPEED),
////                                new InstantCommand(() -> m_intakeArm.setPWM(0), m_intakeArm),
////                                () -> !m_intakeArm.getTopLimitSwitch()
//////                        )
//        );

        m_jack.getJoystickButton(XboxController.Button.kBumperRight)
                .toggleWhenPressed(new IntakeArmMove(m_intakeArm, IntakeArmMove.IntakeArmPosition.TOP, Constants.IntakeArm.SPEED)
                );

        m_jack.getJoystickButton(XboxController.Button.kBumperLeft)
                .toggleWhenPressed(new IntakeArmMove(m_intakeArm, IntakeArmMove.IntakeArmPosition.BOTTOM, Constants.IntakeArm.SPEED));
    }

    public void configureIntakingCommands() {

        // Increment high magazine
        m_jack.getJoystickButton(XboxController.Button.kA).whileHeld(
            new ConditionalCommand(
                new IncrementHighMagazine(m_highMagazine, Constants.Magazine.HIGH_INDEX_BY_ONE_POS),//.withInterrupt(() -> !m_ballCounter.transferIRHasBall()),
                new InstantCommand(() -> {
                    m_highMagazine.setPWM(0);
                }, m_highMagazine),
                () -> {
                    return
                            // !m_ballCounter.entranceIRHasBall() && m_ballCounter.getEntranceLastVal()
                            m_ballCounter.transferIRHasBall();
//                                            (m_ballCounter.getNumBallsHigh() <= Constants.Magazine.HIGH_MAGAZINE_BALL_CAPACITY);
                }
         )).whenReleased(
                new InstantCommand(() -> {
                    m_highMagazine.setPWM(0);
                }, m_highMagazine)
        );

        // Run low magazine
        m_jack.getJoystickButton(XboxController.Button.kA).whileHeld(
            new RunCommand(() -> {
              m_lowMagazine.runVelocityPID(Constants.Magazine.LOW_INTAKE_VELOCITY);
                })
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
                    m_flywheels.setPWM(Constants.Flywheels.FYWHEEL_OUTTAKE_PWM);
                }, m_lowMagazine, m_highMagazine, m_feeder, m_flywheels)
        ).whenReleased(
                new InstantCommand(() -> {
                    m_lowMagazine.setPWM(0);
                    m_highMagazine.setPWM(0);
                    m_feeder.setPWM(0);
                    m_flywheels.setPWM(0);
                })
        );

        m_jack.getJoystickButton((XboxController.Button.kStart)).whileHeld(
                new RunCommand(() -> {
                    m_lowMagazine.setPWM(Constants.Magazine.OUTTAKE_PWM);
                })
        ).whenReleased(
                new InstantCommand(() -> {
                    m_lowMagazine.setPWM(0);
                })
        );
    }

    public void configureShootingCommands() {


//        m_suraj.getJoystickButton(XboxController.Button.kBumperRight).whenPressed(
//                new ConditionalCommand(
//                        new RunCommand(()->m_highMagazine.runVelocityPID(-10)),
//                        null,
//                        ()->!m_ballCounter.exitIRHasBall()
//                )
//        );


//        m_suraj.getJoystickButton(XboxController.Button.kBumperRight).whenPressed(
//                new IncrementHighMagazine(m_highMagazine, Constants.Magazine.HIGH_INDEX_BY_ONE_POS)
//        );

        m_suraj.getJoystickButton(XboxController.Button.kBumperRight).whenPressed(
                new InstantCommand(() -> {



                    m_highMagazine.setPWM(-0.4);
                    m_feeder.setPWM(-0.9);

                })
        );

//        m_suraj.getJoystickButton(XboxController.Button.kY).whenPressed(
//                new InstantCommand(() -> {
//
//                    m_hood.setEncoderPosition(34.9);
//                })
//        );

        m_suraj.getJoystickButton(XboxController.Button.kX).whileHeld(
                new TurretSetAngle(m_turret, () -> {
                    return m_turret.getEncoderPosition()
                            + m_turret.getLimelight().getTargetHorizontalOffset(0)
                            + Deadband.cubicScaledDeadband(
                            m_suraj.getX(GenericHID.Hand.kRight),
                            Constants.OI.XBOX_DEADBAND);
                })
        ).whenReleased(
                new InstantCommand(() -> m_turret.setPWM(0))
        );

        m_suraj.getJoystickButton(XboxController.Button.kStart).whileHeld(
                new RunCommand(() -> {
                    m_feeder.setPWM(-0.9);
                }, m_feeder)
        ).whenReleased(
                new InstantCommand(() -> {
                    m_feeder.setPWM(0);
                }, m_feeder)
        );

        // Increment/feed ball into shooter
        m_suraj.getJoystickButton(XboxController.Button.kBumperRight).whileHeld(
            new ConditionalCommand(new FeedBallToShooter( m_flywheels,  m_lowMagazine,  m_feeder,  m_highMagazine),
                                  new InstantCommand(),
                                 () -> m_flywheels.atVelocitySetpoint()                     
           )).whenReleased(
                new InstantCommand(() -> {
                    m_lowMagazine.setPWM(0);
                    m_highMagazine.setPWM(0);
                    m_feeder.setPWM(0);
                    m_ballCounter.setNumBallsLow(0);
                    m_ballCounter.setNumBallsHigh(0);
                }, m_lowMagazine, m_highMagazine, m_feeder, m_ballCounter
                )
        );

        // Spin up and aim
//        m_suraj.getJoystickButton(XboxController.Button.kBumperLeft).whileHeld(
//                new ParallelCommandGroup(
//                        new Shoot(m_flywheels, m_hood, m_turret.getLimelight(),
//                                () -> -Deadband.cubicScaledDeadband(
//                                        m_suraj.getY(GenericHID.Hand.kLeft),
//                                        Constants.OI.XBOX_DEADBAND),
//                                () -> Constants.Flywheels.RPM_ADJUST * (Deadband.cubicScaledDeadband(
//                                        m_suraj.getTriggerAxis(GenericHID.Hand.kRight),
//                                        Constants.OI.XBOX_DEADBAND)
//                                        - Deadband.cubicScaledDeadband(
//                                        m_suraj.getTriggerAxis(GenericHID.Hand.kLeft),
//                                        Constants.OI.XBOX_DEADBAND)),
//                                () -> Constants.Hood.AUTO_HOOD_MANUAL_ADJUST * -Deadband.cubicScaledDeadband(
//                                        m_suraj.getX(GenericHID.Hand.kRight),
//                                        Constants.OI.XBOX_DEADBAND)
//                        ))).whenReleased(new InstantCommand(() -> {
//            m_flywheels.setPWM(0);
//            m_hood.setPWM(0);
//        }));


        SmartDashboard.putNumber("Velocity RPM", -6000);



        DoubleSupplier flywheelsSetpoint = () -> {
            if (m_suraj.getYButton()) {
                return Constants.Setpoints.INITIATION_LINE.RPM; // close
            } else if (m_suraj.getBButton()) {
                return Constants.Setpoints.CLOSE_TRENCH.RPM;
            } else if (m_suraj.getAButton()) {
                return  Constants.Setpoints.FAR.RPM; // far

            } else {
                return Math.copySign(SmartDashboard.getNumber("Velocity RPM", -6000), -1);
            }

        };

        // shoot left trigger
        m_suraj.getJoystickAxisButton(Axis.kLeftTrigger, 0.2).whileHeld(
                new SetFlywheels(m_flywheels, flywheelsSetpoint)
        ).whenReleased(
                new InstantCommand(
                        () -> {
                            m_flywheels.setPWM(0);
                        }
                )
        );

        // far
//        m_suraj.getJoystickButton(XboxController.Button.kA).whileHeld(
//                new ParallelCommandGroup(
//                        new SetHood(m_hood, () -> Constants.Setpoints.FAR.ANGLE, false)
//                )
//        );

        // close trench
//        m_suraj.getJoystickButton(XboxController.Button.kB).whileHeld(
//                new ParallelCommandGroup(
//                        new SetHood(m_hood, () -> Constants.Setpoints.CLOSE_TRENCH.ANGLE, false)
//                )
//        );

        //zone 1 (green) shot - close shot
        m_suraj.getJoystickButton(XboxController.Button.kY).whileHeld(
                new ParallelCommandGroup(
                        new SetHood(m_hood, () -> Constants.Setpoints.GREEN_ZONE.ANGLE, false)
                )
        );

        //zone 2 (yellow) shot
        m_suraj.getJoystickButton(XboxController.Button.kB).whileHeld(
                new ParallelCommandGroup(
                        new SetHood(m_hood, () -> Constants.Setpoints.YELLOW_ZONE.ANGLE, false)
                )
        );

        //zone 3 (blue) shot
        m_suraj.getJoystickButton(XboxController.Button.kA).whileHeld(
                new ParallelCommandGroup(
                        new SetHood(m_hood, () -> Constants.Setpoints.BLUE_ZONE.ANGLE, false)
                )
        );

        //zone 4 (red) shot
        m_suraj.getJoystickButton(XboxController.Button.kBumperLeft).whileHeld(
                new ParallelCommandGroup(
                        new SetHood(m_hood, () -> Constants.Setpoints.RED_ZONE.ANGLE, false)
                )
        );

        // Turret Seek
//        new ConditionalCommand(
//                new TurretSetAngle(m_turret, () -> {
//                    return m_turret.getEncoderPosition()
//                            + m_turret.getLimelight().getTargetHorizontalOffset(0)
//                            + Deadband.cubicScaledDeadband(
//                            m_suraj.getX(GenericHID.Hand.kRight),
//                            Constants.OI.XBOX_DEADBAND);
//                }),
//                new SequentialCommandGroup(
//                        new TurretSetAngle(m_turret, Constants.Turret.MIN_POSITION + Constants.Turret.BUFFER_ZONE_SIZE, true),
//                        new TurretSetAngle(m_turret, Constants.Turret.MAX_POSITION - Constants.Turret.BUFFER_ZONE_SIZE, true)
//                ).withInterrupt(() -> m_turret.getLimelight().hasValidTarget()),
//                () -> m_turret.getLimelight().hasValidTarget());


    }

    public void configureTurretCommands() {
        // Turret manual control
//        m_turret.setDefaultCommand(
//                new RunCommand(() ->
//                        m_turret.setClampedPWM(
//                                0.7 * Deadband.cubicScaledDeadband(
//                                        m_suraj.getX(GenericHID.Hand.kLeft),
//                                        Constants.OI.XBOX_DEADBAND)
//                        )
//                        , m_turret)
//        );

        m_turret.setDefaultCommand(
                new RunCommand(() -> {
//                    m_turret.runPositionPID(
//                            m_turret.getEncoderPosition() + getAxis(m_suraj, Axis.kLeftX) * Constants.Turret.MANUAL_ANGLE_SCALE
//                    );
                    m_turret.runVelocityPID(
                            getAxis(m_suraj, Axis.kLeftX) * Constants.Turret.MAX_VELOCITY
                    );
                }, m_turret)
        );

        // Field Centric Control
//        m_suraj.getJoystickButton(XboxController.Button.kX).whenHeld(
//                new TurretFieldCentricAdjust(m_turret,
//                        () -> {
//                            return Constants.Turret.TURRET_SPEED * Deadband.cubicScaledDeadband(
//                                    m_suraj.getX(GenericHID.Hand.kLeft),
//                                    Constants.OI.XBOX_DEADBAND);
//                        },



//                        () -> {
//                            return m_drivetrain.getHeading();
//                        }
//                )
//        );

//        m_suraj.getJoystickButton(XboxController.Button.kX).whileHeld(new ConditionalCommand(
//                        new TurretSetAngle(m_turret, () -> {
//                            return m_turret.getEncoderPosition()
//                                    + m_turret.getLimelight().getTargetHorizontalOffset(0)
//                                    + Constants.Turret.AUTO_TURRET_MANUAL_ADJUST * Deadband.cubicScaledDeadband(
//                                    m_suraj.getX(GenericHID.Hand.kRight),
//                                    Constants.OI.XBOX_DEADBAND);
//                        }),
//                new SequentialCommandGroup(
//                        new TurretSetAngle(m_turret, Constants.Turret.MIN_POSITION, true),
//                        new TurretSetAngle(m_turret, Constants.Turret.MAX_POSITION, true)
//                ).withInterrupt(() -> m_turret.getLimelight().hasValidTarget()),
//                        new InstantCommand(() -> m_turret.setPWM(0)),
//                        () -> m_turret.getLimelight().hasValidTarget())
//        ).whenReleased(
//                new InstantCommand(() -> m_turret.setPWM(0))
//        );
    }

    public void configureHoodCommands() {
        m_hood.setDefaultCommand(new RunCommand(
                () -> {
                    m_hood.runVelocityPID(Constants.Hood.HOOD_MAX_VELOCITY
                            * -Deadband.linearScaledDeadband(m_suraj.getY(GenericHID.Hand.kRight), Constants.OI.XBOX_DEADBAND));
                }, m_hood
        ));

//        m_hood.setDefaultCommand(
//                new SetHood(m_hood, () -> {
//                    return m_hood.getEncoderPosition() - getAxis(m_suraj, Axis.kRightY) * Constants.Hood.MANUAL_ANGLE_SCALE;
//                }, false)
//        );

//        m_suraj.getJoystickButton(XboxController.Button.kX).whileHeld(new ConditionalCommand(
//                new SetHood(m_hood, () -> {
//                    return 90 - (90 - m_hood.getEncoderPosition()
//                            - Constants.Robot.LIMELIGHT_ANGLE_FROM_HORIZONTAL
//                            + m_turret.getLimelight().getTargetVerticalOffset(0)
//                            - Deadband.cubicScaledDeadband(
//                            m_suraj.getY(GenericHID.Hand.kRight),
//                            Constants.OI.XBOX_DEADBAND) * Constants.Hood.AUTO_HOOD_MANUAL_ADJUST);
//                }),
//                new InstantCommand(() -> m_hood.setPWM(0)),
//                () -> m_turret.getLimelight().hasValidTarget())
//        ).whenReleased(
//                new InstantCommand(() -> m_hood.setPWM(0))
//        );

    }

    public void configureFlywheelsCommands() {

        m_flywheels.setDefaultCommand(
            new RunCommand(() -> {
                double output = 0;
                double pwm = Deadband.linearScaledDeadband(m_suraj.getTriggerAxis(GenericHID.Hand.kRight), Constants.OI.XBOX_DEADBAND);

                if (pwm != 0) {
                    output = -4000 - pwm * 1000;

                    m_flywheels.setVelocity(output);
                } else {
                    m_flywheels.setPWM(0);
                }

            }, m_flywheels)
    );
//        m_suraj.getJoystickButton(XboxController.Button.kB).whileHeld(
//                new RunCommand(() -> {
//                    m_flywheels.setVelocity(SmartDashboard.getNumber("RPM Setpoint", 0));
//                })
//        ).whenReleased(new InstantCommand(() -> m_flywheels.setPWM(0)));
    }

    public void configureClimberCommands() {
//        m_jack.getJoystickButton(XboxController.Button.kY).whileHeld(
//                new RunCommand(() -> {
//                    m_climber.setPWM(Deadband.linearScaledDeadband(m_jack.getY(GenericHID.Hand.kRight), Constants.OI.XBOX_DEADBAND));
//                })
//        );

        // new TurretSetAngle(m_turret, m_turret.getEncoderPosition() > 0? 90: -90, false);

        m_jack.getJoystickButton(XboxController.Button.kY).whileHeld(new ClimberCommands (m_climber, m_turret))
               .whenReleased(
                new InstantCommand(() -> m_climber.setPWM(0)));
    }

    public Command getAutonomousCommand() {
        // temporary!

//        m_autoCommand = new RunCommand(() -> {

//        });

        m_autoCommand = new AutoCommand(m_flywheels, m_turret, m_feeder, m_highMagazine, m_lowMagazine, m_drivetrain, m_hood);
        return m_autoCommand;
    }


    public void configureTuning() {
        SmartDashboard.putBoolean(Constants.Turret.ZERO_TURRET_LABEL, false);
        SmartDashboard.putBoolean(Constants.Drivetrain.RESET_GYRO_LABEL, false);
        SmartDashboard.putNumber(Constants.TUNE_LAYER_LABEL, 0);
        SmartDashboard.putBoolean(Constants.PID_ENABLE_LABEL, false);
        SmartDashboard.putBoolean(Constants.TUNE_ENABLE_LABEL, false);
        SmartDashboard.putBoolean(Constants.RESET_PID_LABEL, false);
        SmartDashboard.putBoolean("Reset PID", false);

        m_tuneChooser.addOption("Low Magazine", m_lowMagazine);
        m_tuneChooser.addOption("High Magazine", m_highMagazine);
        m_tuneChooser.addOption("Turret", m_turret);
        m_tuneChooser.addOption("Feeder", m_feeder);
        m_tuneChooser.addOption("Hood", m_hood);
        m_tuneChooser.addOption("Flywheels", m_flywheels);

        SmartDashboard.putData("Tune Chooser", m_tuneChooser);
    }

    public void testInit() {
        this.gameInit();
        SmartDashboard.putNumber(Constants.TUNE_LAYER_LABEL, 0);
        SmartDashboard.putBoolean(Constants.PID_ENABLE_LABEL, false);
        SmartDashboard.putBoolean(Constants.TUNE_ENABLE_LABEL, false);
    }

    public void testPeriodic() {


        if (SmartDashboard.getBoolean(Constants.RESET_PID_LABEL, false)) {
            m_tuneChooser.getSelected().resetPIDs();
            SmartDashboard.putBoolean(Constants.RESET_PID_LABEL, false);
        }

        if (SmartDashboard.getBoolean(Constants.TUNE_ENABLE_LABEL, false)) {
            if (SmartDashboard.getBoolean(Constants.PID_ENABLE_LABEL, false)) {
                m_tuneChooser.getSelected().tunePeriodic((int) SmartDashboard.getNumber(Constants.TUNE_LAYER_LABEL, 0));
            } else {
                m_tuneChooser.getSelected().setPWM(-getAxis(m_jack, Axis.kLeftY));
            }
        } else {

            /**
             * Zeroing logic
             */

            if (SmartDashboard.getBoolean(Constants.Turret.ZERO_TURRET_LABEL, false)) {
                m_turret.resetEncoderPosition(0);
                SmartDashboard.putBoolean(Constants.Turret.ZERO_TURRET_LABEL, false);
            }

            if (SmartDashboard.getBoolean(Constants.Drivetrain.RESET_GYRO_LABEL, false)) {
                m_drivetrain.setHeading(0);
                SmartDashboard.putBoolean(Constants.Drivetrain.RESET_GYRO_LABEL, false);
            }

            /**
             * Hood zeroing
             */
            if (m_jack.getXButton()) {
                if (!m_hood.getForwardLimitSwitch()) {
                    m_hood.runUnclampedVelocityPID(100);
                } else {
                    m_hood.forceZero();
                }
            } else {
                m_hood.setPWM(0);
            }

            /**
             * Turret zeroing
             */
            if (m_jack.getAButton()) {
                if (!m_turret.getReverseLimitSwitch()) {
                    m_turret.runUnclampedVelocityPID(-10);
                } else {
                    m_turret.resetEncoderPosition(Constants.Turret.MIN_POSITION);
                }
            } else {
                m_turret.setPWM(0);
            }
        }
    }

    /**
     * Convenience function for getting a deadbanded axis
     *
     * @param controller which controller to poll from
     * @param axis       which axis to use
     * @return the value of the specified controller axis
     */
    private double getAxis(WL_XboxController controller, Axis axis) {
        return Deadband.linearScaledDeadband(controller.getRawAxis(axis.value), Constants.OI.XBOX_DEADBAND);
    }
}

