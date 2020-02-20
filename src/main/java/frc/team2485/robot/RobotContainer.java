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
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.team2485.WarlordsLib.oi.Deadband;
import frc.team2485.WarlordsLib.oi.WL_XboxController;
import frc.team2485.robot.commands.IntakeArmMove;
import frc.team2485.robot.subsystems.IntakeArm;

public class RobotContainer {

    private WL_XboxController m_jack;

    private IntakeArm m_intakeArm;

    private Command m_autoCommand;

    public RobotContainer() {

        m_intakeArm = new IntakeArm();

        m_jack = new WL_XboxController(Constants.OI.JACK_PORT);

        configureCommands();
    }

    private void configureCommands() {
        m_jack.getJoystickButton(XboxController.Button.kA).whenHeld(new IntakeArmMove(m_intakeArm, IntakeArmMove.IntakeArmPosition.BOTTOM, Constants.IntakeArm.SPEED));
        m_jack.getJoystickButton(XboxController.Button.kB).whenHeld(new IntakeArmMove(m_intakeArm, IntakeArmMove.IntakeArmPosition.TOP, Constants.IntakeArm.SPEED));

        m_jack.getJoystickButton(XboxController.Button.kBumperLeft).whenHeld(
                new RunCommand(()-> m_intakeArm.setPWM(Deadband.linearScaledDeadband(m_jack.getY(GenericHID.Hand.kRight), 0.1)))
        );
    }

    public Command getAutonomousCommand() {
        // temporary!
        m_autoCommand = new RunCommand(() -> {
        });

        return m_autoCommand;
    }
}
