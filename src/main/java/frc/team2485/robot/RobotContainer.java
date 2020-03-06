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
import frc.team2485.robot.commandgroups.Feed;
import frc.team2485.robot.commandgroups.IntakeBalls;
import frc.team2485.robot.commands.*;
import frc.team2485.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;

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

    private SendableChooser<Tunable> m_tuneChooser;
    private SendableChooser<Command> m_autoChooser;

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

        m_autoChooser = new SendableChooser<>();
        m_tuneChooser = new SendableChooser<>();

        this.configureCommands();
        this.configureAutos();
        if (Constants.TUNE_MODE) {
            this.configureTuning();
        }
    }

    private void configureCommands() {

        // drivetrain
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

        // intake arm
        m_intakeArm.setDefaultCommand(
                new PerpetualCommand(
                        new RunCommand(
                                () -> {
                                    if (!m_intakeArm.getTopLimitSwitch()) {
                                        m_intakeArm.setPWM(Constants.IntakeArm.Setpoints.DEFAULT_PWM);
                                    } else {
                                        m_intakeArm.setPWM(0);
                                    }
                                }, m_intakeArm
                        )
                )
        );

//        m_jack.getJoystickButton(XboxController.Button.kBumperRight)
//                .whenHeld(new IntakeArmMove(m_intakeArm, IntakeArmMove.IntakeArmPosition.TOP, Constants.IntakeArm.DEFAULT_PWM)
//                );

        m_jack.getJoystickButton(XboxController.Button.kBumperLeft)
                .whileHeld(new IntakeArmMove(m_intakeArm, IntakeArmMove.IntakeArmPosition.BOTTOM, Constants.IntakeArm.Setpoints.DEFAULT_PWM));

        // hood manual
        m_hood.setDefaultCommand(new RunCommand(
                () -> {
                    m_hood.runVelocityPID(Constants.Hood.HOOD_MAX_VELOCITY
                            * -Deadband.linearScaledDeadband(m_suraj.getY(GenericHID.Hand.kRight), Constants.OI.XBOX_DEADBAND));
                }, m_hood
        ));

        m_jack.getJoystickButton(XboxController.Button.kStart).whenPressed(
                new InstantCommand(() -> {
                    m_ballCounter.setNumBallsLow(0);
                    m_ballCounter.setNumBallsHigh(0);
                })
        );

        // intaking
        m_jack.getJoystickButton(XboxController.Button.kA).toggleWhenPressed(new IntakeBalls(m_lowMagazine, m_highMagazine, m_ballCounter));

        // Outtake
        m_jack.getJoystickButton(XboxController.Button.kB).whileHeld(
                new RunCommand(() -> {
                    m_lowMagazine.setPWM(Constants.Magazine.Setpoints.OUTTAKE_PWM);
                    m_highMagazine.setPWM(Constants.Magazine.Setpoints.OUTTAKE_PWM);
                    m_feeder.setPWM(Constants.Feeder.Setpoints.OUTTAKE_PWM);
                    m_flywheels.setPWM(Constants.Flywheels.FYWHEEL_OUTTAKE_PWM);
                }, m_lowMagazine, m_highMagazine, m_feeder, m_flywheels)
        ).whenReleased(
                new InstantCommand(() -> {
                    m_lowMagazine.setPWM(0);
                    m_highMagazine.setPWM(0);
                    m_feeder.setPWM(0);
                    m_flywheels.setPWM(0);
                }, m_lowMagazine, m_highMagazine, m_feeder, m_flywheels)
        );

        // seek and align
        m_suraj.getJoystickButton(XboxController.Button.kX).whileHeld(
//                new TurretLimelightAlign(m_turret, () -> Constants.Turret.AUTO_TURRET_MANUAL_ADJUST * getAxis(m_suraj, Axis.kLeftX))
                new ConditionalCommand(
                        new TurretLimelightAlign(m_turret, () -> Constants.Turret.AUTO_TURRET_MANUAL_ADJUST * getAxis(m_suraj, Axis.kLeftX)),
                        new SequentialCommandGroup(
                                new TurretSetAngle(m_turret, Constants.Turret.MIN_POSITION + Constants.Turret.BUFFER_ZONE_SIZE, true),
                                new TurretSetAngle(m_turret, Constants.Turret.MAX_POSITION - Constants.Turret.BUFFER_ZONE_SIZE, true)
                        ).withInterrupt(() -> m_turret.getLimelight().hasValidTarget()),
                        () -> m_turret.getLimelight().hasValidTarget()
                )
        );

        // Increment/feed ball into shooter
        m_suraj.getJoystickButton(XboxController.Button.kBumperRight).whileHeld(
                new SequentialCommandGroup(
                        new RunCommand(
                                () -> {
                                    m_highMagazine.runVelocityPID(Constants.Magazine.Setpoints.HIGH_MAGAZINE_FEED_VELOCITY);
                                }, m_highMagazine
                        ).withInterrupt(
                                () -> m_ballCounter.exitIRHasBall()
                        ).andThen(
                                new InstantCommand(
                                        () -> {
                                            m_highMagazine.setPWM(0);
                                        }, m_highMagazine
                                )
                        ),
                        new WaitUntilCommand(() -> m_flywheels.atVelocitySetpoint()),
                        new Feed(m_feeder, m_highMagazine, m_lowMagazine, m_ballCounter, 8)
                )
        ).whenReleased(
                new InstantCommand(
                        () -> {
                            m_lowMagazine.setPWM(0);
                            m_highMagazine.setPWM(0);
                            m_feeder.setPWM(0);
                            m_ballCounter.setNumBallsLow(0);
                            m_ballCounter.setNumBallsHigh(0);
                        }
                )
        );

        // control panel setpoint
        m_suraj.getJoystickButton(XboxController.Button.kA).whileHeld(
                new ParallelCommandGroup(
                        new FlywheelsSetRPM(m_flywheels,
                                () -> Constants.ShootingSetpoints.CONTROL_PANEL.RPM + (getAxis(m_suraj, Axis.kRightTrigger) - getAxis(m_suraj, Axis.kLeftTrigger)) * Constants.Flywheels.RPM_ADJUST
                        ),
                        new HoodSetAngle(m_hood,
                                () -> Constants.ShootingSetpoints.CONTROL_PANEL.HOOD_ANGLE + getAxis(m_suraj, Axis.kRightY) * Constants.Hood.Setpoints.HOOD_ADJUST),
                        new TurretLimelightAlign(m_turret,
                                () -> Constants.Turret.AUTO_TURRET_MANUAL_ADJUST * getAxis(m_suraj, Axis.kLeftX))
                )
        );

        // initiation line setpoint
        m_suraj.getJoystickButton(XboxController.Button.kY).whileHeld(
                new ParallelCommandGroup(
                        new FlywheelsSetRPM(m_flywheels,
                                () -> Constants.ShootingSetpoints.INITIATION_LINE.RPM + (getAxis(m_suraj, Axis.kRightTrigger) - getAxis(m_suraj, Axis.kLeftTrigger)) * Constants.Flywheels.RPM_ADJUST
                        ),
                        new HoodSetAngle(m_hood,
                                () -> Constants.ShootingSetpoints.INITIATION_LINE.HOOD_ANGLE + getAxis(m_suraj, Axis.kRightY) * Constants.Hood.Setpoints.HOOD_ADJUST),
                        new TurretLimelightAlign(m_turret, () -> Constants.Turret.AUTO_TURRET_MANUAL_ADJUST * getAxis(m_suraj, Axis.kLeftX))
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

        // turret
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

        // flywheels manual
        m_flywheels.setDefaultCommand(
                new RunCommand(() -> {
                    double output = 0;
                    double pwm = Deadband.linearScaledDeadband(m_suraj.getTriggerAxis(GenericHID.Hand.kRight), Constants.OI.XBOX_DEADBAND);

                    if (pwm != 0) {
                        output = -3000 - pwm * 3000;

                        m_flywheels.setVelocity(output);
                    } else {
                        m_flywheels.setPWM(0);
                    }

                }, m_flywheels)
        );

//        SmartDashboard.putNumber("Velocity RPM", -6000);

        // set rpm
        m_suraj.getJoystickButton(XboxController.Button.kBumperLeft).whileHeld(
                new RunCommand(() -> {
                    double pwm = getAxis(m_suraj, Axis.kRightTrigger) - getAxis(m_suraj, Axis.kLeftTrigger);

                    double output = -4000 - pwm * 2000;

                    m_flywheels.setVelocity(output);

                }, m_flywheels)
//                new FlywheelsSetRPM(m_flywheels, () -> SmartDashboard.getNumber("Velocity RPM", -6000))
        ).whenReleased(
                new InstantCommand(
                        () -> m_flywheels.setPWM(0)
                )
        );

        // turret field centric control
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


//        m_jack.getJoystickButton(XboxController.Button.kY).whileHeld(
//                new RunCommand(() -> {
//                    m_climber.setPWM(Deadband.linearScaledDeadband(m_jack.getY(GenericHID.Hand.kRight), Constants.OI.XBOX_DEADBAND));
//                })
//        );

        // climbing
        m_jack.getJoystickButton(XboxController.Button.kY).whileHeld(
                new InstantCommand(() -> m_climber.setPWM(Constants.Climber.DEFAULT_PWM))).whenReleased(
                new InstantCommand(() -> m_climber.setPWM(0))
        );

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
    }

    public void resetPIDs() {
        m_turret.resetPIDs();
        m_hood.resetPID();
        m_flywheels.resetPIDs();
        m_highMagazine.resetPIDs();
        m_lowMagazine.resetPIDs();
        m_intakeArm.resetPIDs();
        m_feeder.resetPIDs();
    }

    public Command getAutonomousCommand() {

        m_autoCommand = m_autoChooser.getSelected();

        return m_autoCommand;
    }

    private void configureAutos() {
        m_autoChooser.addOption("Basic Auto",
                new ParallelCommandGroup(
                        new InstantCommand(
                                () -> {
                                    m_drivetrain.resetEncoders(0, 0);
                                    m_drivetrain.setHeading(0);
                                }
                        ),
                        // Always align turret
                        new TurretLimelightAlign(m_turret),
                        new SequentialCommandGroup(
                                new ParallelDeadlineGroup(
                                        // Deadline
                                        new SequentialCommandGroup(
                                                new WaitUntilCommand(() -> m_hood.atPositionSetpoint() && m_flywheels.atVelocitySetpoint() && m_turret.atPositionSetpoint()),
                                                new Feed(m_feeder, m_highMagazine, m_lowMagazine, m_ballCounter, 4)
                                        ),
                                        new FlywheelsSetRPM(m_flywheels, Constants.ShootingSetpoints.INITIATION_LINE.RPM),
                                        new HoodSetAngle(m_hood, Constants.ShootingSetpoints.INITIATION_LINE.HOOD_ANGLE)
                                ),
                                new ParallelDeadlineGroup(
                                        new WaitCommand(2),
                                        new RunCommand(
                                                () -> {
                                                    m_drivetrain.setPWM(0.3);
                                                }
                                        )
                                )
                        )
                )
        );

        SmartDashboard.putData("Auto Chooser", m_autoChooser);
    }

    private void configureTuning() {
        m_tuneChooser.addOption("Low Magazine", m_lowMagazine);
        m_tuneChooser.addOption("High Magazine", m_highMagazine);
        m_tuneChooser.addOption("Turret", m_turret);
        m_tuneChooser.addOption("Feeder", m_feeder);
        m_tuneChooser.addOption("Hood", m_hood);
        m_tuneChooser.addOption("Flywheels", m_flywheels);

        SmartDashboard.putBoolean(Constants.Turret.ZERO_TURRET_LABEL, false);
        SmartDashboard.putBoolean(Constants.Drivetrain.RESET_GYRO_LABEL, false);
        SmartDashboard.putNumber(Constants.TUNE_LAYER_LABEL, 0);
        SmartDashboard.putBoolean(Constants.RESET_PID_LABEL, false);
        SmartDashboard.putBoolean(Constants.PID_ENABLE_LABEL, false);
        SmartDashboard.putBoolean(Constants.TUNE_ENABLE_LABEL, false);
        SmartDashboard.putData("Tune Chooser", m_tuneChooser);
    }

    public void testInit() {
        m_tuneChooser.getSelected().resetPIDs();
    }

    public void testPeriodic() {

        if (SmartDashboard.getBoolean(Constants.RESET_PID_LABEL, false)) {
            m_tuneChooser.getSelected().resetPIDs();
            SmartDashboard.putBoolean(Constants.RESET_PID_LABEL, false);
        }

        if (Constants.TUNE_MODE && SmartDashboard.getBoolean(Constants.TUNE_ENABLE_LABEL, false)) {
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
            if (m_jack.getYButton()) {
                if (!m_turret.getReverseLimitSwitch()) {
                    m_turret.runUnclampedVelocityPID(-10);
                } else {
                    m_turret.resetEncoderPosition(Constants.Turret.MIN_POSITION);
                }
            } else {
                m_turret.setPWM(0);
            }

            /**
             * Climber reset
             */
            m_climber.setPWM(-getAxis(m_jack, Axis.kRightY));
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

