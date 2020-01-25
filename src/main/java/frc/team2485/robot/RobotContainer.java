/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team2485.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.team2485.WarlordsLib.oi.Deadband;
import frc.team2485.WarlordsLib.oi.WL_XboxController;
import frc.team2485.robot.commands.TurretLimelightAlign;
import frc.team2485.robot.subsystems.Drivetrain;

public class RobotContainer {

    private WL_XboxController m_jack;
    private WL_XboxController m_suraj;

    private Drivetrain m_drivetrain;
    private frc.team2485.robot.subsystems.Turret m_turret;

    private Command m_autoCommand;

    public RobotContainer() {

        m_drivetrain = new Drivetrain();
        m_turret = new frc.team2485.robot.subsystems.Turret();

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

        m_turret.setDefaultCommand(
                new ConditionalCommand(
                        new TurretLimelightAlign(m_turret),
                        null, // Turret odometry align
                        () -> m_turret.getLimelight().hasValidTarget()
                )
        );

        // Manual control
        m_suraj.getJoystickButton(XboxController.Button.kY).whenPressed(
                new RunCommand(() -> {
                    m_turret.setPWM(
                            Deadband.cubicScaledDeadband(
                                    m_suraj.getX(GenericHID.Hand.kLeft),
                                    Constants.OI.XBOX_DEADBAND)
                    );
                })
        );

        // Field Centric Control
        m_suraj.getJoystickButton(XboxController.Button.kX).whenPressed(
                new RunCommand(() -> {
                    m_turret.setPositionPID(
                            m_turret.getEncoderPosition()
                                    + m_drivetrain.getHeading()
                                    + Constants.Turret.TURRET_SPEED * Deadband.cubicScaledDeadband(
                                    m_suraj.getX(GenericHID.Hand.kLeft),
                                    Constants.OI.XBOX_DEADBAND)
                    );
                })
        );
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
}
