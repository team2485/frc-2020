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
import frc.team2485.robot.commands.*;
import frc.team2485.robot.commands.paths.*;
import frc.team2485.robot.subsystems.*;
import edu.wpi.first.wpiutil.math.MathUtil;

import frc.team2485.robot.subsystems.Drivetrain;
import com.revrobotics.CANSparkMax;

import java.util.function.DoubleSupplier;

public class RobotContainer {

    private WL_XboxController m_jack;
    private WL_XboxController m_suraj;

    private Drivetrain m_drivetrain;
    private LowMagazine m_lowMagazine;
    private HighMagazine m_highMagazine;

    private Feeder m_feeder;
    private Flywheels m_flywheels;
    private Hood m_hood;
    private Climber m_climber;
    private Turret m_turret;
    private Intake m_intake;

    private GSPathChooser m_GSPathChooser;

    private Command m_autoCommand;

    private boolean intakeDown;

    private boolean m_turretToggle;

    private SendableChooser<Tunable> m_tuneChooser;

    public RobotContainer() {

        m_jack = new WL_XboxController(Constants.OI.JACK_PORT);
        m_suraj = new WL_XboxController(Constants.OI.SURAJ_PORT);

        m_drivetrain = new Drivetrain();
        m_lowMagazine = new LowMagazine();
        m_highMagazine = new HighMagazine();
        m_feeder = new Feeder();
        m_flywheels = new Flywheels();
        m_hood = new Hood(m_feeder.getHoodEncoder());
        m_turret = new Turret();
        m_climber = new Climber();
        m_intake = new Intake();
        m_GSPathChooser = new GSPathChooser(m_drivetrain, m_lowMagazine, m_turret.getLimelight());

        m_turretToggle = false; 

        m_tuneChooser = new SendableChooser<Tunable>();

        this.configureCommands();
    }

    public void gameInit() {
        m_turret.resetPIDs();
        m_hood.resetPID();
        m_flywheels.resetPIDs();
        m_highMagazine.resetPIDs();
        m_lowMagazine.resetPIDs();
        m_feeder.resetPIDs();
        m_intake.resetPIDs();

    }

    private void configureCommands() {

        this.configureDrivetrainCommands();
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

        //reset ball count
        m_suraj.getJoystickButton(XboxController.Button.kStart).whenPressed(new InstantCommand(() -> {
            m_flywheels.fullCount();
        }));

        m_suraj.getJoystickAxisButton(Axis.kLeftTrigger, 0.2).whenPressed(new InstantCommand(()->
        {m_flywheels.zeroCount();}));



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

    public void configureIntakingCommands() {

        //Intake lowering (CONFIGURE CONSTANTS)
//        m_jack.getJoystickButton(XboxController.Button.kY).whenPressed(
//            new ConditionalCommand(
//                new SequentialCommandGroup(
//                    new InstantCommand(()-> m_lowMagazine.setPWM(Constants.Intake.LOWERING_PWM)),
//                    new WaitCommand(Constants.Intake.LOWERING_TIME),
//                    new InstantCommand(()-> m_lowMagazine.setPWM(0)),
//                    new InstantCommand(()->{intakeDown = true;})
//                ),
//                new InstantCommand(()->{}),
//                ()-> {return intakeDown = false;}
//            )
//        );

        //holding command to lower intake
        m_jack.getJoystickButton(XboxController.Button.kY).whileHeld(
                new InstantCommand(() ->    
                    m_lowMagazine.setPWM(Constants.Intake.LOWERING_PWM)))
                        .whenReleased(
                                new InstantCommand(() -> m_lowMagazine.setPWM(0
                                ))
                        );
        
        // Increment high magazine
        m_jack.getJoystickButton(XboxController.Button.kA).whileHeld(
                new ConditionalCommand(
                        new IncrementHighMagazine(m_highMagazine, Constants.Magazine.HIGH_INDEX_BY_ONE_POS),
                        new InstantCommand(() -> {
                            m_highMagazine.setPWM(0);
                        }, m_highMagazine),
                        () -> {
                            return m_flywheels.transferIRHasBall();
                        }
                )
        ).whenReleased(
                new InstantCommand(() -> {
                    m_highMagazine.setPWM(0);
                }, m_highMagazine)
        );

        // Run low magazine and intake
        m_jack.getJoystickButton(XboxController.Button.kA).whileHeld(
            new RunCommand(() -> {
                m_lowMagazine.setPWM(Constants.Magazine.LOW_BELT_INTAKE_PWM);
                //m_lowMagazine.runVelocityPID(Constants.Magazine.LOW_INTAKE_VELOCITY);
                //m_lowMagazine.setPWM(-0.4);
                m_intake.runVelocityPID(Constants.Intake.X_VELOCITY, Constants.Intake.Z_VELOCITY);
                //m_intake.setPWM(-0.5, -0.5);

            })
        ).whenReleased(
                new InstantCommand(
                        () -> {
                                m_lowMagazine.setPWM(0);
                                m_intake.setPWM(0);
                })
        );

        m_jack.getJoystickButton(XboxController.Button.kB).whileHeld(
                new RunCommand(() -> {
                    m_lowMagazine.setPWM(Constants.Magazine.OUTTAKE_PWM);
                    m_highMagazine.setPWM(Constants.Magazine.OUTTAKE_PWM);
                    m_feeder.setPWM(Constants.Feeder.OUTTAKE_PWM);
                    m_flywheels.setPWM(Constants.Flywheels.FYWHEEL_OUTTAKE_PWM);
                    m_intake.setPWM(Constants.Intake.OUTTAKE_PWM);
                }, m_lowMagazine, m_highMagazine, m_feeder, m_flywheels)
        ).whenReleased(
                new InstantCommand(() -> {
                    m_lowMagazine.setPWM(0);
                    m_highMagazine.setPWM(0);
                    m_feeder.setPWM(0);
                    m_flywheels.setPWM(0);
                    m_intake.setPWM(0);
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

        m_suraj.getJoystickButton(XboxController.Button.kBumperRight).whenPressed(
                new InstantCommand(() -> {
                    m_highMagazine.setPWM(-0.4);
                    m_feeder.setPWM(-0.9);

                })
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

        //Index and increment into shooter once
        m_suraj.getJoystickButton(XboxController.Button.kStickRight).whenPressed(
            new SequentialCommandGroup( 
                new ConditionalCommand(
                    new InstantCommand(()->{System.out.println("apple");}),
                    new SequentialCommandGroup(
                        //This is where the indexing is applied 
                        //Supposed to move the magaine up by enough to put every ball at the top
                        //May need fiddling to get the numbers right (new constant)
                        new IncrementHighMagazine(m_highMagazine, MathUtil.clamp(Constants.Magazine.HIGH_MAGAZINE_BALL_CAPACITY - m_flywheels.getBalls(), 0, Constants.Magazine.HIGH_MAGAZINE_BALL_CAPACITY) * Constants.Magazine.HIGH_INCREMENT_TOP),
                        new InstantCommand(()->{m_flywheels.updateBallPosition(true);
                        System.out.println("Banana: " + String.valueOf(Constants.Magazine.HIGH_MAGAZINE_BALL_CAPACITY - m_flywheels.getBalls()));})
                        
                    ),
                    ()->{return m_flywheels.getBallPosition();}
                ), 
                new SequentialCommandGroup(
                    new WaitUntilCommand(() -> m_flywheels.atVelocitySetpoint()),
                    new InstantCommand(
                            () -> {
                                m_lowMagazine.setPWM(-0.5);
                                m_feeder.setPWM(-0.9); //change
                                m_flywheels.incrementBalls(false);
                            }, m_lowMagazine, m_feeder
                    ),
                    new WaitCommand(0.5),
                    new IncrementHighMagazine(m_highMagazine, Constants.Magazine.PUSH_IN_INCREMENT)
                    )
            ));

            m_suraj.getJoystickButton(XboxController.Button.kStickLeft).whenPressed(
                new InstantCommand(() -> {
                    m_lowMagazine.setPWM(0);
                    m_highMagazine.setPWM(0);
                    m_feeder.setPWM(0);
                    //m_ballCounter.setNumBallsLow(0);
                    //m_ballCounter.setNumBallsHigh(0);
                }, m_lowMagazine, m_highMagazine, m_feeder
                )
            );

        // Increment/feed ball into shooter (this is for shooting all the balls one after another, generally,)
        m_suraj.getJoystickButton(XboxController.Button.kBumperRight).whileHeld(
                new SequentialCommandGroup(
                    new ConditionalCommand(
                        new InstantCommand(()->{}),
                        new SequentialCommandGroup(
                            //This is where the indexing is applied 
                            //Supposed to move the magaine up by enough to put every ball at the top
                            //May need fiddling to get the numbers right (new constant)
                            new IncrementHighMagazine(m_highMagazine, (Constants.Magazine.HIGH_MAGAZINE_BALL_CAPACITY - m_flywheels.getBalls()) * Constants.Magazine.HIGH_INCREMENT_TOP),
                            new InstantCommand(()->{m_flywheels.updateBallPosition(true);})
                        ),
                        ()->{return m_flywheels.getBallPosition();}
                    ),
                    new ConditionalCommand(
                        new SequentialCommandGroup(
                                new WaitUntilCommand(() -> m_flywheels.atVelocitySetpoint()),
                                new InstantCommand(
                                        () -> {
                                            m_lowMagazine.setPWM(-0.5);
                                            m_feeder.setPWM(-0.9); //change
                                            m_flywheels.incrementBalls(false);
                                        }, m_lowMagazine, m_feeder
                                ),
                                new IncrementHighMagazine(m_highMagazine, Constants.Magazine.HIGH_INCREMENT_TOP)
                        ),
                        new InstantCommand(),
                        () -> m_flywheels.atVelocitySetpoint()
                    )
                )
        ).whenReleased(
                new InstantCommand(() -> {
                    m_lowMagazine.setPWM(0);
                    m_highMagazine.setPWM(0);
                    m_feeder.setPWM(0);
                }, m_lowMagazine, m_highMagazine, m_feeder
                )
        );

        // DoubleSupplier flywheelsSetpoint = () -> {
        //     if (m_suraj.getYButton()) {
        //         return Constants.Setpoints.INITIATION_LINE.RPM; // close
        //     } else if (m_suraj.getBButton()) {
        //         return Constants.Setpoints.CLOSE_TRENCH.RPM;
        //     } else if (m_suraj.getAButton()) {
        //         return  Constants.Setpoints.FAR.RPM; // far

        //     } else {
        //         return Math.copySign(SmartDashboard.getNumber("Velocity RPM", -3000), -1);
        //     }

        // };

        // // shoot left trigger
        // m_suraj.getJoystickAxisButton(Axis.kLeftTrigger, 0.2).whileHeld(
        //         new SetFlywheels(m_flywheels, () -> {return -3000;})
        // ).whenReleased(
        //         new InstantCommand(
        //                 () -> {
        //                     m_flywheels.setPWM(0);
        //                 }
        //         )
        // );
        
        //zone 1 (green) shot - close shot
        m_suraj.getJoystickButton(XboxController.Button.kY).whenPressed(
                // new ParallelCommandGroup(
                //         new SetHood(m_hood, () -> Constants.Setpoints.GREEN_ZONE.ANGLE),
                //         new SetFlywheels(m_flywheels, () -> Constants.Setpoints.GREEN_ZONE.RPM)
                // )
                //new InstantCommand(() -> {
                    new SetHood(m_hood, () -> Constants.Setpoints.GREEN_ZONE.ANGLE)
                //})
        );

        //zone 2 (yellow) shot
        m_suraj.getJoystickButton(XboxController.Button.kB).whenPressed(
                // new ParallelCommandGroup(
                //         new SetHood(m_hood, () -> Constants.Setpoints.YELLOW_ZONE.ANGLE),
                //         new SetFlywheels(m_flywheels, () -> Constants.Setpoints.YELLOW_ZONE.RPM)
                // )
                //new InstantCommand(() -> {
                    new SetHood(m_hood, () -> Constants.Setpoints.YELLOW_ZONE.ANGLE)
                //})
        );

        //zone 3 (blue) shot
        m_suraj.getJoystickButton(XboxController.Button.kA).whenPressed(
                // new ParallelCommandGroup(
                //         new SetHood(m_hood, () -> Constants.Setpoints.BLUE_ZONE.ANGLE),
                //         new SetFlywheels(m_flywheels, () -> Constants.Setpoints.BLUE_ZONE.RPM)
                // )
                //new InstantCommand(() -> {
                    new SetHood(m_hood, () -> Constants.Setpoints.BLUE_ZONE.ANGLE)
                //})
        );

        //zone 4 (red) shot
        m_suraj.getJoystickButton(XboxController.Button.kBumperLeft).whenPressed(
                // new ParallelCommandGroup(
                //         new SetHood(m_hood, () -> Constants.Setpoints.RED_ZONE.ANGLE),
                //         new SetFlywheels(m_flywheels, () -> Constants.Setpoints.RED_ZONE.RPM)
                // )
                //new InstantCommand(() -> {
                    new SetHood(m_hood, () -> Constants.Setpoints.RED_ZONE.ANGLE)
                //})
        );
    }

    public void configureTurretCommands() {
        // m_turret.setDefaultCommand(
        //     new ConditionalCommand(
        //         new TurretSetAngle(m_turret, () -> {
        //             return m_turret.getEncoderPosition()
        //                     + m_turret.getLimelight().getTargetHorizontalOffset(0)
        //                     + Deadband.cubicScaledDeadband(
        //                     m_suraj.getX(GenericHID.Hand.kRight),
        //                     Constants.OI.XBOX_DEADBAND);
        //         }),
        //         new TurretWithController(m_turret, m_suraj),
        //         ()-> {return m_turretToggle;}
        //     )
        // );
        m_turret.setDefaultCommand(new TurretWithController(m_turret, m_suraj));
    
        

      //  m_suraj.getJoystickButton(XboxController.Button.kX).toggleWhenPressed(
      //      new StartEndCommand(
      //          () -> {m_turretToggle = true;},
      //          () -> {m_turretToggle = false;}
      //      ));
    
    }

    public void configureHoodCommands() {
        m_hood.setDefaultCommand(new RunCommand(
                () -> {
                    m_hood.runVelocityPID(Constants.Hood.HOOD_MAX_VELOCITY
                            * -Deadband.linearScaledDeadband(m_suraj.getY(GenericHID.Hand.kRight), Constants.OI.XBOX_DEADBAND));
                }, m_hood
        ));
    }

    public void configureFlywheelsCommands() {
        
        m_flywheels.setDefaultCommand(
                new RunCommand(() -> {
                    double output = 0;
                    double pwmRight = Deadband.linearScaledDeadband(m_suraj.getTriggerAxis(GenericHID.Hand.kRight), Constants.OI.XBOX_DEADBAND);
                    double pwmLeft = Deadband.linearScaledDeadband(m_suraj.getTriggerAxis(GenericHID.Hand.kLeft), Constants.OI.XBOX_DEADBAND);

                    if ( pwmRight != 0) {
                            output = -4000 - pwmRight* 1000;

                        m_flywheels.setVelocity(output);
                    } else if (pwmLeft != 0) {
                        output = -3450 - pwmLeft * 1000;

                        m_flywheels.setVelocity(output);
                    } else {
                        m_flywheels.setPWM(0);
                    }

                }, m_flywheels)
        );
    }

    public void autonomousInit() {
        m_drivetrain.setIdleMode(CANSparkMax.IdleMode.kBrake);
    }

    public Command getAutonomousCommand() {

        // double tx = m_turret.getLimelight().getTargetHorizontalOffset(0);
        // double startPointY = -1 * (Constants.Autonomous.POWER_PORT_X_POS + (Constants.Autonomous.INITIATION_LINE_X * Math.tan( Math.toRadians(tx)))); //check sign of tx
        // double startPointX = Constants.Autonomous.INITIATION_LINE_X;

        //lower intake 
        
        // run intake rollers and magazine
            Command runIntake = new RunCommand(() -> {
                m_lowMagazine.setPWM(Constants.Magazine.LOW_BELT_INTAKE_PWM);
                m_intake.runVelocityPID(Constants.Intake.X_VELOCITY, Constants.Intake.Z_VELOCITY);
                ;
        });

        // // increment high magazine to inake
        Command highIntake = new RunCommand(()-> {m_highMagazine.setPWM(-0.1);});

        //return new ParallelRaceGroup(new BBluePath(m_drivetrain), new ParallelCommandGroup(runIntake, highIntake));
        return new LowerIntake(m_lowMagazine).andThen(new ParallelRaceGroup(m_GSPathChooser.getPath(), new ParallelCommandGroup(runIntake, highIntake))); 
        //return new ARedPath(m_drivetrain);
        //return new InstantCommand(m_GSPathChooser::evaluate);
        //return new ParallelCommandGroup(runIntake, highIntake);
        //return new BackPath(m_drivetrain);
        //return new ParallelRaceGroup(m_GSPathChooser.getPath(), new ParallelCommandGroup(runIntake, highIntake));
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
        m_tuneChooser.addOption("Intake", m_intake);

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

