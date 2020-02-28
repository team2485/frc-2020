/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team2485.robot;

import com.ctre.phoenix.sensors.PigeonIMU;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;

import frc.team2485.WarlordsLib.oi.Deadband;
import frc.team2485.WarlordsLib.oi.WL_XboxController;
import frc.team2485.robot.commands.IncrementHighMagazine;
import frc.team2485.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.team2485.WarlordsLib.Limelight;
import frc.team2485.robot.commands.SetHood;
import frc.team2485.robot.commands.Shoot;

import frc.team2485.robot.commands.TurretFieldCentricAdjust;
import frc.team2485.robot.commands.TurretSetAngle;
import frc.team2485.robot.commands.IntakeArmMove;
import frc.team2485.robot.subsystems.Drivetrain;

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

    SetHood setHood;
    //    private Drivetrain m_drivetrain;
    private Turret m_turret;

    //Temporary
    private PigeonIMU pigeon;

    private IntakeArm m_intakeArm;

    private Command m_autoCommand;

    public RobotContainer() {

        m_drivetrain = new Drivetrain();
        m_lowMagazine = new LowMagazine();
        m_highMagazine = new HighMagazine(m_lowMagazine::getTransferIR);

        m_feeder = new Feeder();
        m_flywheels = new Flywheels();
        m_hood = new Hood(m_feeder.getHoodEncoder());


        m_turret = new Turret();
        m_intakeArm = new IntakeArm();

        m_climber = new Climber();



        m_jack = new WL_XboxController(Constants.OI.JACK_PORT);
        m_suraj = new WL_XboxController(Constants.OI.SURAJ_PORT);
        m_jack = new WL_XboxController(Constants.OI.JACK_PORT);
        m_suraj = new WL_XboxController(Constants.OI.SURAJ_PORT);

        pigeon = new PigeonIMU(1);

        configureCommands();
    }

    private void configureCommands() {



        m_suraj.getJoystickButton(XboxController.Button.kB).whenHeld(
                new RunCommand(() -> {
                    m_hood.setPWM(Deadband.linearScaledDeadband(m_suraj.getY(GenericHID.Hand.kRight), Constants.OI.XBOX_DEADBAND));
                })
        ).whenReleased(
                new RunCommand(() -> {
                    m_hood.setPWM(0);
                })
        );



        /*
        DRIVETRAIN
         */
        m_drivetrain.setDefaultCommand(
                new RunCommand(() -> {
                    m_drivetrain.curvatureDrive(
                            Deadband.cubicScaledDeadband(
                                    m_jack.getTriggerAxis(GenericHID.Hand.kRight) - m_jack.getTriggerAxis(GenericHID.Hand.kLeft),
                                    Constants.OI.XBOX_DEADBAND),
                            Deadband.cubicScaledDeadband(
                                    m_jack.getX(GenericHID.Hand.kLeft),
                                    Constants.OI.XBOX_DEADBAND
                            ),
                            m_jack.getXButton());
                }, m_drivetrain)
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


                            SmartDashboard.putNumber("Pigeon Heading", -pigeon.getFusedHeading());
                            return -pigeon.getFusedHeading();
                        }
                )
        );

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
        m_suraj.getJoystickButton(XboxController.Button.kA).whileHeld(
                new SequentialCommandGroup(
                        new InstantCommand(() -> m_lowMagazine.setPWM(-0.5))
                                .alongWith(new IncrementHighMagazine(m_highMagazine, Constants.Magazine.HIGH_INDEX_BY_ONE_POS)),
                        new WaitCommand(Constants.Magazine.NORMAL_BALL_INCREMENT_TIMEOUT)
                )
        ).whenReleased(
                new InstantCommand(() -> m_lowMagazine.setPWM(0))
        );

        //set both feeder and shooter to 0.2

//        m_suraj.getJoystickButton(XboxController.Button.kY).whileHeld(
//                new InstantCommand(()->m_feeder.setPWM(-0.3)).alongWith(
//                        new InstantCommand(()->m_flywheels.setPWM(-0.3))
//                )
//        ).whenReleased(
//                new InstantCommand(()->m_flywheels.setPWM(0)).alongWith(
//                        new InstantCommand(()->m_feeder.setPWM(0))
//                )
//        );

        SmartDashboard.putData("reset encoder", new InstantCommand(() -> {
            m_highMagazine.resetEncoder(0);
        }));

        m_suraj.getJoystickButton(XboxController.Button.kBumperLeft).whenHeld(new ConditionalCommand(
                new TurretSetAngle(m_turret, m_turret.getMinAngle(), true),
                new TurretSetAngle(m_turret, m_turret.getMaxAngle(), true),
                () -> m_turret.getEncoderPosition() > 0
        ));

        // Seek
        m_suraj.getJoystickButton(XboxController.Button.kA).whileHeld(
                new ConditionalCommand(
                        new TurretSetAngle(m_turret, () -> {
                            return m_turret.getEncoderPosition() + m_turret.getLimelight().getTargetHorizontalOffset(0);
                        }),
                        new SequentialCommandGroup(
                                new TurretSetAngle(m_turret, Constants.Turret.MIN_POSITION + Constants.Turret.BUFFER_ZONE_SIZE, true),
                                new TurretSetAngle(m_turret, Constants.Turret.MAX_POSITION - Constants.Turret.BUFFER_ZONE_SIZE, true)
                        ).withInterrupt(() -> m_turret.getLimelight().hasValidTarget()),
                        () -> m_turret.getLimelight().hasValidTarget())
        );

        // Toggle Limelight LED
        m_suraj.getJoystickButton(XboxController.Button.kBack).whenPressed(new InstantCommand(() -> {
            m_turret.getLimelight().toggleLed();
        }));

        m_suraj.getJoystickAxisButton(XboxController.Axis.kLeftTrigger, Constants.OI.SURAJ_LTRIGGER_THRESHOLD).whileHeld(
                new Shoot(m_flywheels, m_hood, m_turret.getLimelight(), () -> {
                    return -m_suraj.getY(GenericHID.Hand.kRight);
                }).alongWith(
                        new InstantCommand(() -> m_feeder.setPWM(-0.3))));

        m_jack.getJoystickButton(XboxController.Button.kA).whileHeld(
                new ConditionalCommand(
                        new InstantCommand(() -> {
//                            m_lowMagazine.setPWM(Constants.Magazine.LOW_BELT_PWM);
                            m_lowMagazine.setVelocity(-30);
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
                        () ->
                                m_lowMagazine.setPWM(0)
                ));

        m_jack.getJoystickButton(XboxController.Button.kB).whileHeld(
                new InstantCommand(() -> {
                    m_lowMagazine.setPWM(0.3);
                    m_highMagazine.setPWM(0.3);
                })
        ).whenReleased(
                new InstantCommand(
                        () -> {
                            m_lowMagazine.setPWM(0);
                            m_highMagazine.setPWM(0);
                        }

                ));


        m_jack.getJoystickButton(XboxController.Button.kA).whileHeld(
                new ConditionalCommand(
                        new IncrementHighMagazine(m_highMagazine, Constants.Magazine.HIGH_INDEX_BY_ONE_POS),
                        new InstantCommand(() -> {
                            m_highMagazine.setPWM(0);
                        }),
                        () -> {
                            return m_highMagazine.getTransferIR() && m_highMagazine.getNumBalls() < Constants.Magazine.HIGH_MAGAZINE_BALL_CAPACITY;
                        }
                )
        ).whenReleased(
                new InstantCommand(() -> {
                    m_highMagazine.setPWM(0);
                })
        );

        m_climber.setDefaultCommand(new RunCommand(() -> {
            m_climber.setPWM(Deadband.linearScaledDeadband(m_jack.getY(GenericHID.Hand.kRight), Constants.OI.XBOX_DEADBAND));
        }, m_climber));



        /*
        INTAKE ARM
         */
        m_jack.getJoystickButton(XboxController.Button.kA)
                .whenHeld(new IntakeArmMove(m_intakeArm, IntakeArmMove.IntakeArmPosition.BOTTOM, Constants.IntakeArm.SPEED));
        m_jack.getJoystickButton(XboxController.Button.kY)
                .whenHeld(new IntakeArmMove(m_intakeArm, IntakeArmMove.IntakeArmPosition.TOP, Constants.IntakeArm.SPEED));
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
            pigeon.setFusedHeading(0);
            SmartDashboard.putBoolean(Constants.Drivetrain.RESET_GYRO_LABEL, false);
        }


        if (Constants.TUNE_MODE) {
            boolean enable = SmartDashboard.getBoolean(Constants.TUNE_ENABLE_LABEL, false);
            if (enable) {
                m_lowMagazine.tunePeriodic();
//                m_highMagazine.tunePeriodic();
            } else {
                m_lowMagazine.setPWM(-Deadband.linearScaledDeadband(m_jack.getY(GenericHID.Hand.kRight), Constants.OI.XBOX_DEADBAND));
                m_highMagazine.setPWM(-Deadband.linearScaledDeadband(m_jack.getY(GenericHID.Hand.kLeft), Constants.OI.XBOX_DEADBAND));
//                m_turret.setUnclampedPWM(Deadband.linearScaledDeadband(m_jack.getX(GenericHID.Hand.kLeft), Constants.OI.XBOX_DEADBAND));
//                m_feeder.setPWM(Deadband.linearScaledDeadband(m_suraj.getY(GenericHID.Hand.kLeft), Constants.OI.XBOX_DEADBAND));
//                m_flywheels.setPWM(-Deadband.linearScaledDeadband(m_suraj.getY(GenericHID.Hand.kRight), Constants.OI.XBOX_DEADBAND));
//                m_hood.setPWM(-Deadband.linearScaledDeadband(m_jack.getY(GenericHID.Hand.kLeft), Constants.OI.XBOX_DEADBAND));
            }
        }
    }


    //All the data the widgets need in networktables for them to work
    public void widgetNetworkTables() {
        //for the Heading Widget
        SmartDashboard.putNumber("Heading/encoder",m_turret.getEncoderPosition()); //In Radians, add this to the Robot's Gyro position
        SmartDashboard.putNumber("Heading/gyro",m_drivetrain.getHeading()); //In Degrees, make sure this is reset at the beginning

        //Data for the shooter widget is in the Shoot command
    }


    //All the data the widgets need in networktables for them to work
    public void widgetNetworkTables() {
        //for the Heading Widget
        SmartDashboard.putNumber("Heading/encoder",m_turret.getEncoderPosition()); //In Radians, add this to the Robot's Gyro position
        SmartDashboard.putNumber("Heading/gyro",m_drivetrain.getHeading()); //In Degrees, make sure this is reset at the beginning

        //Data for the shooter widget is in the Shoot command
    }
}

