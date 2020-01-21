/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team2485.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.team2485.WarlordsLib.oi.Deadband;
import frc.team2485.WarlordsLib.oi.WL_XboxController;
import frc.team2485.robot.subsystems.Drivetrain;

public class RobotContainer {

    private WL_XboxController m_jack;

    private Drivetrain m_drivetrain;

    private Command m_autoCommand;

    public RobotContainer() {

        m_drivetrain = new Drivetrain();

        m_jack = new WL_XboxController(Constants.OI.JACK_PORT);

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
