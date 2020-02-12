/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team2485.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.*;
import frc.team2485.WarlordsLib.oi.Deadband;
import frc.team2485.WarlordsLib.oi.WL_XboxController;
import frc.team2485.robot.commands.IncrementHighMagazine;
import frc.team2485.robot.commands.IncrementLowMagazine;
import frc.team2485.robot.subsystems.Drivetrain;
import frc.team2485.robot.subsystems.HighMagazine;
import frc.team2485.robot.subsystems.LowMagazine;

public class RobotContainer {

    private WL_XboxController m_jack;
    private WL_XboxController m_suraj;

    private Drivetrain m_drivetrain;
    private LowMagazine m_lowMagazine;
    private HighMagazine m_highMagazine;

    private Command m_autoCommand;

    public RobotContainer() {

        m_drivetrain = new Drivetrain();
        m_lowMagazine= new LowMagazine();
        m_highMagazine = new HighMagazine(m_lowMagazine::getTransferIR);

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

        m_highMagazine.setDefaultCommand(new ConditionalCommand(new IncrementHighMagazine(m_highMagazine), null, () -> {
            return m_highMagazine.getTransferIR() && m_highMagazine.getNumBalls() < 4;
        }));

        m_lowMagazine.setDefaultCommand(new ConditionalCommand(new RunCommand(() -> m_lowMagazine.setPWM(Constants.Magazine.LOW_BELT_PWM)), null, () -> {
            return m_highMagazine.getNumBalls() >= Constants.Magazine.HIGH_MAGAZINE_BALL_CAPACITY && m_lowMagazine.getTransferIR();
        }));

        m_suraj.getJoystickButton(XboxController.Button.kA).whenPressed(new RunCommand(() -> {
            m_lowMagazine.setPWM(Constants.Magazine.FAST_OUTTAKE_PWM);
            m_highMagazine.setPWM(Constants.Magazine.FAST_OUTTAKE_PWM);
        }));

        m_suraj.getJoystickButton(XboxController.Button.kB).whenHeld(new RunCommand(()-> {
            m_lowMagazine.setPWM(Deadband.linearScaledDeadband(m_suraj.getTriggerAxis(GenericHID.Hand.kRight),Constants.OI.XBOX_DEADBAND));
        }));

        m_suraj.getJoystickButton(XboxController.Button.kBumperRight).whileHeld(new IncrementLowMagazine(m_lowMagazine)
                                                                                .alongWith(new IncrementHighMagazine(m_highMagazine)
                                                                                .andThen(new WaitCommand(Constants.Magazine.NORMAL_OUTTAKE_TIMEOUT))));


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
