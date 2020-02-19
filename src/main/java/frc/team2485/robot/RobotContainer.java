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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.team2485.WarlordsLib.Limelight;
import frc.team2485.WarlordsLib.oi.Deadband;
import frc.team2485.WarlordsLib.oi.WL_XboxController;
import frc.team2485.robot.commands.shooter.Shoot;
import frc.team2485.robot.subsystems.Drivetrain;
import frc.team2485.robot.subsystems.shooter.Feeder;
import frc.team2485.robot.subsystems.shooter.Flywheels;
import frc.team2485.robot.subsystems.shooter.Hood;

public class RobotContainer {

    private WL_XboxController m_jack;
    private WL_XboxController m_suraj;

    private Drivetrain m_drivetrain;

    private Feeder m_feeder;
    private Flywheels m_flywheels;
    private Hood m_hood;
    private Limelight m_limelight;

    private Command m_autoCommand;

    public RobotContainer() {

        m_drivetrain = new Drivetrain();

        m_feeder = new Feeder();
        m_flywheels = new Flywheels();
        m_hood = new Hood(m_flywheels.getHoodEncoder());
        m_limelight = new Limelight();

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

        m_suraj.getJoystickAxisButton(XboxController.Axis.kLeftTrigger, Constants.OI.SURAJ_LTRIGGER_THRESHOLD).whenHeld(
                new Shoot(m_flywheels, m_hood, m_limelight, ()-> {
                    return - m_suraj.getY(GenericHID.Hand.kRight);
                } ));

        m_jack.getJoystickButton(XboxController.Button.kA).whenHeld(
                new RunCommand(()->
                {m_feeder.setPwM(Deadband.linearScaledDeadband(
                        m_jack.getY(GenericHID.Hand.kRight), 0.1));
                    SmartDashboard.putNumber("JOYSTICK PWM SETPOINT",Deadband.linearScaledDeadband(
                            m_jack.getY(GenericHID.Hand.kRight), 0.1) );}));





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
