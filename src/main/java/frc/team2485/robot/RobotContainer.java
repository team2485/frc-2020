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

import frc.team2485.WarlordsLib.oi.Deadband;
import frc.team2485.WarlordsLib.oi.WL_XboxController;
import frc.team2485.robot.commands.IncrementHighMagazine;
import frc.team2485.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.team2485.robot.commands.SetHood;
import frc.team2485.robot.commands.Shoot;

import frc.team2485.robot.commands.TurretSetAngle;
import frc.team2485.robot.commands.IntakeArmMove;
import frc.team2485.robot.subsystems.Drivetrain;

import java.time.Instant;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.function.BooleanSupplier;

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

        this.configureCommands();
    }
    
    public Turret getTurret() {
        return m_turret;
    }
    public Drivetrain getDrivetrain() {
        return m_drivetrain;
    }
    public Hood getHood() {
        return m_hood;
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
                            -Deadband.cubicScaledDeadband(
                                    m_jack.getTriggerAxis(GenericHID.Hand.kRight) - m_jack.getTriggerAxis(GenericHID.Hand.kLeft),
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
//                new IntakeArmMove(m_intakeArm, IntakeArmMove.IntakeArmPosition.TOP, Constants.IntakeArm.SPEED)
//        );

        m_jack.getJoystickButton(XboxController.Button.kBumperLeft)
                .whenHeld(new IntakeArmMove(m_intakeArm, IntakeArmMove.IntakeArmPosition.TOP, Constants.IntakeArm.SPEED)
                );

        m_jack.getJoystickButton(XboxController.Button.kBumperRight)
                .whenHeld(new IntakeArmMove(m_intakeArm, IntakeArmMove.IntakeArmPosition.BOTTOM, Constants.IntakeArm.SPEED));
//        .whenReleased(new IntakeArmMove(m_intakeArm, IntakeArmMove.IntakeArmPosition.TOP, Constants.IntakeArm.SPEED));
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
                        new IncrementHighMagazine(m_highMagazine, Constants.Magazine.HIGH_INDEX_BY_ONE_POS),//.withInterrupt(() -> !m_ballCounter.getTransferIR()),
                        new InstantCommand(() -> {
                            m_highMagazine.setPWM(0);
                        }),
                        () -> {
                            return
                                    // !m_ballCounter.getEntranceIR() && m_ballCounter.getEntranceLastVal()
                                    m_ballCounter.getTransferIR() &&
                                            (m_ballCounter.getNumBallsHigh() <= Constants.Magazine.HIGH_MAGAZINE_BALL_CAPACITY);
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
//                            m_lowMagazine.setPWM(Constants.Magazine.LOW_BELT_INTAKE_PWM);
                            m_lowMagazine.runVelocityPID(-50);

                        }),
                        new InstantCommand(() -> {
                            m_lowMagazine.setPWM(0);
                        }),
                        () -> {
                            return !(m_ballCounter.getNumBallsHigh() >= Constants.Magazine.HIGH_MAGAZINE_BALL_CAPACITY
                                    && m_ballCounter.getEntranceIR());
                            //return true;

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
                    m_flywheels.setPWM(Constants.Flywheels.FYWHEEL_OUTTAKE_PWM);
                })
        ).whenReleased(
                new InstantCommand(() -> {
                    m_lowMagazine.setPWM(0);
                    m_highMagazine.setPWM(0);
                    m_feeder.setPWM(0);
                    m_flywheels.setPWM(0);
                })
        );
    }

    public void configureShootingCommands() {


//        m_suraj.getJoystickButton(XboxController.Button.kBumperRight).whenPressed(
//                new ConditionalCommand(
//                        new RunCommand(()->m_highMagazine.runVelocityPID(-10)),
//                        null,
//                        ()->!m_ballCounter.getExitIR()
//                )
//        );
        // Increment ball into shooter
        m_suraj.getJoystickButton(XboxController.Button.kBumperRight).whileHeld(
                new ConditionalCommand(
                        new SequentialCommandGroup(
                                new InstantCommand(() -> {
                                    m_lowMagazine.setPWM(-0.5);
                                    m_feeder.setPWM(-0.8);
                                }).alongWith(
                                        new IncrementHighMagazine(m_highMagazine, Constants.Magazine.HIGH_INDEX_BY_ONE_POS),
                                        new WaitCommand(Constants.Magazine.NORMAL_BALL_INCREMENT_TIMEOUT)
                                )),
                                new InstantCommand(),
                                () -> m_flywheels.getLeftEncoderVelocity() < -1000 && m_flywheels.getRightEncoderVelocity() < -1000
                        )).whenReleased(
                        new InstantCommand(() -> {
                            m_lowMagazine.setPWM(0);
                            m_highMagazine.setPWM(0);
                            m_feeder.setPWM(0);
                            m_ballCounter.setNumBallsLow(0);
                            m_ballCounter.setNumBallsHigh(0);
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
                                () -> Constants.Hood.AUTO_HOOD_MANUAL_ADJUST * -Deadband.cubicScaledDeadband(
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
                new TurretSetAngle(m_turret,
                        () -> {
                            double setpoint = m_turret.getEncoderPosition();
                            double pwm = getAxis(m_suraj, Axis.kLeftX);

                            setpoint += pwm * Constants.Turret.MANUAL_ANGLE_SCALE;

//                            if (pwm > 0) {
//                                setpoint += (m_turret.getMaxAngle() - m_turret.getEncoderPosition()) * pwm;
//                            } else if (pwm < 0) {
//                                setpoint -= Math.abs((m_turret.getMinAngle() - m_turret.getEncoderPosition()) * pwm);
//                            }

                            return setpoint;
                        }
                )
        );

//      //   Limelight align
//        m_suraj.getJoystickButton(XboxController.Button.kX).whenHeld(
//                new TurretSetAngle(m_turret, () -> {
//                    return m_turret.getEncoderPosition() + m_turret.getLimelight().getTargetHorizontalOffset(0);
//                })
//        );


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

        m_suraj.getJoystickButton(XboxController.Button.kX).whileHeld(new ConditionalCommand(
                        new TurretSetAngle(m_turret, () -> {
                            return m_turret.getEncoderPosition()
                                    + m_turret.getLimelight().getTargetHorizontalOffset(0)
                                    + Constants.Turret.AUTO_TURRET_MANUAL_ADJUST * Deadband.cubicScaledDeadband(
                                    m_suraj.getX(GenericHID.Hand.kRight),
                                    Constants.OI.XBOX_DEADBAND);
                        }),
//                new SequentialCommandGroup(
//                        new TurretSetAngle(m_turret, Constants.Turret.MIN_POSITION, true),
//                        new TurretSetAngle(m_turret, Constants.Turret.MAX_POSITION, true)
//                ).withInterrupt(() -> m_turret.getLimelight().hasValidTarget()),
                        new InstantCommand(() -> m_turret.setPWM(0)),
                        () -> m_turret.getLimelight().hasValidTarget())
        ).whenReleased(
                new InstantCommand(() -> m_turret.setPWM(0))
        );
    }

    public void configureHoodCommands() {
        m_hood.setDefaultCommand(new RunCommand(
                () -> {
                    m_hood.setPWM(-Deadband.linearScaledDeadband(m_suraj.getY(GenericHID.Hand.kRight), Constants.OI.XBOX_DEADBAND));
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
        SmartDashboard.putNumber("RPM Setpoint", 0);

        m_flywheels.setDefaultCommand(
                new RunCommand(() -> {
                    double output = 0;
                    double pwm = Deadband.linearScaledDeadband(m_suraj.getTriggerAxis(GenericHID.Hand.kRight), Constants.OI.XBOX_DEADBAND);

                    if (pwm != 0) {
                        output = -1000 - pwm * 5000;

                        m_flywheels.setVelocity(output);
                    } else {
                        m_flywheels.setPWM(0);
                    }

                }, m_flywheels)
        );

        m_suraj.getJoystickButton(XboxController.Button.kB).whileHeld(
                new RunCommand(() -> {
                    m_flywheels.setVelocity(SmartDashboard.getNumber("RPM Setpoint", 0));
                })
        ).whenReleased(new InstantCommand(() -> m_flywheels.setPWM(0)));
    }

    public void configureClimberCommands() {
//        m_jack.getJoystickButton(XboxController.Button.kY).whileHeld(
//                new RunCommand(() -> {
//                    m_climber.setPWM(Deadband.linearScaledDeadband(m_jack.getY(GenericHID.Hand.kRight), Constants.OI.XBOX_DEADBAND));
//                })
//        );

        m_jack.getJoystickButton(XboxController.Button.kY).whileHeld(
                new InstantCommand(() -> m_climber.setPWM(Constants.Climber.DEFAULT_PWM)));
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

        m_turret.resetPID();
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
//                m_lowMagazine.tunePeriodic(0);
                //m_highMagazine.tunePeriodic(0);
//               m_turret.tunePeriodic(1);
                m_hood.tunePeriodic(1);
//                m_flywheels.tunePeriodic(0);
//                m_feeder.tunePeriodic(0);
            } else {
//                m_flywheels.setPWM(0);
//                m_highMagazine.setPWM(0);
//                m_lowMagazine.setPWM(-getAxis(m_jack, Axis.kLeftY));
             //   m_highMagazine.setPWM(-getAxis(m_jack, Axis.kLeftY));
      //         m_turret.setPWM(getAxis(m_suraj, Axis.kLeftX));
//                m_feeder.setPWM(-getAxis(m_jack, Axis.kLeftY));
//                m_intakeArm.setPWM(getAxis(m_jack, Axis.kRightY));
                m_hood.setPWM(-Deadband.linearScaledDeadband(m_jack.getY(GenericHID.Hand.kLeft), Constants.OI.XBOX_DEADBAND));

//                m_feeder.setPWM(-getAxis(m_jack, Axis.kLeftY));
//                m_flywheels.setPWM(-Deadband.linearScaledDeadband(m_suraj.getY(GenericHID.Hand.kRight), Constants.OI.XBOX_DEADBAND));
//                m_hood.setPWM(-Deadband.linearScaledDeadband(m_suraj.getY(GenericHID.Hand.kLeft), Constants.OI.XBOX_DEADBAND));

            }

            if (!enable || m_jack.getBumper(GenericHID.Hand.kRight) || m_suraj.getBumper(GenericHID.Hand.kRight)) {
//                m_lowMagazine.setPWM(-getAxis(m_jack, Axis.kLeftY));
//                m_highMagazine.setPWM(-getAxis(m_jack, Axis.kLeftY));
//                m_turret.setPWM(getAxis(m_suraj, Axis.kLeftX));
//                m_feeder.setPWM(-getAxis(m_jack, Axis.kLeftY));
//                m_intakeArm.setPWM(getAxis(m_jack, Axis.kRightY));
//                m_flywheels.setPWM(-Deadband.linearScaledDeadband(m_suraj.getY(GenericHID.Hand.kRight), Constants.OI.XBOX_DEADBAND));
//                m_hood.setPWM(-Deadband.linearScaledDeadband(m_suraj.getY(GenericHID.Hand.kLeft), Constants.OI.XBOX_DEADBAND));
            }
        }
    }

    private double getAxis(WL_XboxController controller, Axis axis) {
        return Deadband.linearScaledDeadband(controller.getRawAxis(axis.value), Constants.OI.XBOX_DEADBAND);
    }

    //All the data the widgets need in networktables for them to work
    public void widget1NetworkTables() {
        //for the Heading Widget
        SmartDashboard.putNumber("Heading/encoder",m_turret.getEncoderPosition()); //In Radians, add this to the Robot's Gyro position
        SmartDashboard.putNumber("Heading/gyro",m_drivetrain.getHeading()); //In Degrees, make sure this is reset at the beginning

        //Data for the shooter widget is in the Shoot command
    }
    //All the data the widgets need in networktables for them to work
    public void widget2NetworkTables() {
        
        double vfy = -m_suraj.getY(GenericHID.Hand.kRight);
        double ty = m_turret.getLimelight().getTargetVerticalOffset(Constants.Robot.LIMELIGHT_TY_DEFAULT_VALUE) + Constants.Shooter.LIMELIGHT_ANGLE_FROM_HORIZONTAL; //gets vertical angle from m_limelight
        double xDist = Shoot.getX(ty, Constants.Robot.HEIGHT_FROM_LL_TO_PORT); //finds x distance (horizontal) to port
        double v0y = Shoot.getv0y(vfy, Constants.Robot.HEIGHT_FROM_LL_TO_PORT, Constants.Shooter.GRAVITY_ACCELERATION_CONSTANT); //finds initial y velocity based on final y velocity and height changes
        double timeOfTrajectory = Shoot.gettimeOfTraj(v0y, vfy, Constants.Shooter.GRAVITY_ACCELERATION_CONSTANT, Constants.Robot.HEIGHT_FROM_SHOOTER_TO_PORT); //finds time of trajectory based on y velocities, distances, and accelerations
        double vfx = Shoot.getvfX(xDist, timeOfTrajectory); //finds final x velocity from time of trajectory and distance traversed
        double v0x = Shoot.getv0xFromVfx(timeOfTrajectory, vfx, Constants.PowerCell.POWER_CELL_DRAG_COEFF, Constants.PowerCell.POWER_CELL_MASS);
        //taken from Shoot.java
        double initialVelocity = Math.sqrt(v0x*v0x+v0y*v0y);
        SmartDashboard.putNumber("Shooter/pitch",m_hood.getEncoderPosition()); // shouldn't we just get this from v0x & v0y?
        SmartDashboard.putNumber("Shooter/iv",initialVelocity);
    }
    public void sendWidget1data(double encoderVal, double gyroVal) { //for testing
        SmartDashboard.putNumber("Heading/encoder",encoderVal); //In Radians, add this to the Robot's Gyro position
        SmartDashboard.putNumber("Heading/gyro",gyroVal); //In Degrees, make sure this is reset at the beginning
    }
    public void sendWidget2data(double pitchVal, double ivVal) { //for testing
        SmartDashboard.putNumber("Shooter/pitch",pitchVal); // shouldn't we just get this from v0x & v0y?
        SmartDashboard.putNumber("Shooter/iv",ivVal);
    }
}

