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
import frc.team2485.WarlordsLib.Limelight;
import frc.team2485.WarlordsLib.oi.Deadband;
import frc.team2485.WarlordsLib.oi.WL_XboxController;
import frc.team2485.robot.commands.TurretFieldCentricAdjust;
import frc.team2485.robot.commands.TurretSetAngle;
import frc.team2485.robot.commands.TurretLimelightAlign;
import frc.team2485.robot.subsystems.Turret;

public class RobotContainer {

    private WL_XboxController m_jack;
    private WL_XboxController m_suraj;

    //    private Drivetrain m_drivetrain;
    private Turret m_turret;

    //Temporary
    private PigeonIMU pigeon;

    private Command m_autoCommand;

    public RobotContainer() {

//        m_drivetrain = new Drivetrain();
        m_turret = new Turret();

        m_jack = new WL_XboxController(Constants.OI.JACK_PORT);
        m_suraj = new WL_XboxController(Constants.OI.SURAJ_PORT);

        pigeon = new PigeonIMU(0);

        configureCommands();
    }

    private void configureCommands() {
//        m_drivetrain.setDefaultCommand(new RunCommand(() -> {
//                    m_drivetrain.curvatureDrive(
//                            Deadband.cubicScaledDeadband(
//                                    m_jack.getTriggerAxis(GenericHID.Hand.kRight)
//                                            - m_jack.getTriggerAxis(GenericHID.Hand.kLeft),
//                                    Constants.OI.XBOX_DEADBAND),
//                            Deadband.cubicScaledDeadband(
//                                    m_jack.getX(GenericHID.Hand.kLeft),
//                                    Constants.OI.XBOX_DEADBAND),
//                            m_jack.getXButton());
//                }, m_drivetrain)
//        );

        m_turret.setDefaultCommand(
                new RunCommand(() ->
                        m_turret.setPWM(
                                Deadband.cubicScaledDeadband(
                                        m_suraj.getX(GenericHID.Hand.kLeft),
                                        Constants.OI.XBOX_DEADBAND)
                        )
                        , m_turret)
        );

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
                            double p = -pigeon.getFusedHeading();

                            p += 180;

                            while (p < 0) {
                                p += 360;
                            }

                            p %= 360;


                            p -= 180;

                            SmartDashboard.putNumber("Pigeon heading", p);


                            return p ;
                        }
                )
        );

        // Seek
        m_suraj.getJoystickButton(XboxController.Button.kA).whileHeld(
                new ConditionalCommand(new TurretSetAngle(m_turret, () -> {
                    return m_turret.getEncoderPosition() + m_turret.getLimelight().getTargetHorizontalOffset(0);
                }),
                        new SequentialCommandGroup(
                                new TurretSetAngle(m_turret, Constants.Turret.MIN_POSITION + Constants.Turret.BUFFER_ZONE_SIZE, true),
                                new TurretSetAngle(m_turret, Constants.Turret.MAX_POSITION - Constants.Turret.BUFFER_ZONE_SIZE, true)
                        ).withInterrupt(()->m_turret.getLimelight().hasValidTarget()),
                        () -> m_turret.getLimelight().hasValidTarget()));

        m_suraj.getJoystickButton(XboxController.Button.kBack).whenPressed(new InstantCommand(() -> {
            m_turret.getLimelight().setLedMode(Limelight.LedMode.OFF);
        }));

        m_suraj.getJoystickButton(XboxController.Button.kStart).whenPressed(new InstantCommand(() -> {
            m_turret.getLimelight().setLedMode(Limelight.LedMode.ON);
        }));

        m_suraj.getJoystickButton(XboxController.Button.kBumperRight).whenPressed(new InstantCommand(() -> {
            m_turret.getLimelight().setLedMode(Limelight.LedMode.BLINK);
        }));

        SmartDashboard.putData("Zero Turret", new InstantCommand(() ->
                m_turret.setEncoderPosition(0)
        ));

        SmartDashboard.putData("Zero Pigeon", new InstantCommand(() ->
                pigeon.setFusedHeading(0)
        ));

        if (Constants.DEBUG_MODE) {
            configTurretTuningCommands();
        }


    }


    private void configTurretTuningCommands() {

        SmartDashboard.putNumber("Turret Setpoint", 0);

        SmartDashboard.putData("Turret PID Command", new TurretSetAngle(m_turret, () -> SmartDashboard.getNumber("Turret Setpoint", 0)));

    }

    public Command getAutonomousCommand() {
        // temporary!
        m_autoCommand = new RunCommand(() -> {

        });

        return m_autoCommand;
    }
}
