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
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.team2485.WarlordsLib.Limelight;
import frc.team2485.WarlordsLib.SmartDashboardHelper;
import frc.team2485.WarlordsLib.oi.Deadband;
import frc.team2485.WarlordsLib.oi.WL_XboxController;
import frc.team2485.robot.commands.shooter.SetHood;
import frc.team2485.robot.commands.shooter.Shoot;
import frc.team2485.robot.subsystems.Feeder;
import frc.team2485.robot.subsystems.Flywheels;
import frc.team2485.robot.subsystems.Hood;

public class RobotContainer {

    private WL_XboxController m_jack;
    private WL_XboxController m_suraj;

    private Feeder m_feeder;
    private Flywheels m_flywheels;
    private Hood m_hood;
    private Limelight m_limelight;


    SetHood setHood;

    private Command m_autoCommand;

    public RobotContainer() {

        m_feeder = new Feeder();
        m_flywheels = new Flywheels();
        m_hood = new Hood(m_feeder.getHoodEncoder());
        m_limelight = new Limelight();
        m_limelight.setLedMode(Limelight.LedMode.OFF);

        m_jack = new WL_XboxController(Constants.OI.JACK_PORT);
        m_suraj = new WL_XboxController(Constants.OI.SURAJ_PORT);

        configureCommands();
    }

    private void configureCommands() {

        m_suraj.getJoystickAxisButton(XboxController.Axis.kLeftTrigger, Constants.OI.SURAJ_LTRIGGER_THRESHOLD).whenHeld(
                new Shoot(m_flywheels, m_hood, m_limelight, () -> {
                    return -m_suraj.getY(GenericHID.Hand.kRight);
                }));

        m_jack.getJoystickButton(XboxController.Button.kA).whenHeld(
                new RunCommand(() ->
                        m_feeder.setPWM(Deadband.linearScaledDeadband(
                                m_jack.getY(GenericHID.Hand.kRight), Constants.OI.XBOX_DEADBAND)))
        ).whenReleased(
                new RunCommand(() -> {
                    m_feeder.setPWM(0);
                })
        );


        m_jack.getJoystickButton(XboxController.Button.kB).whenHeld(
                new RunCommand(() -> {
                    m_hood.setPWM(Deadband.linearScaledDeadband(m_jack.getY(GenericHID.Hand.kRight), Constants.OI.XBOX_DEADBAND));
                })
        ).whenReleased(
                new RunCommand(() -> {
                    m_hood.setPWM(0);
                })
        );

        setHood = new SetHood(m_hood, 0);
    }

    public Command getAutonomousCommand() {
        // temporary!
        m_autoCommand = new RunCommand(() -> {
        });

        return m_autoCommand;
    }

    public void testInit() {

        SmartDashboard.putBoolean("Tune Enable", false);
        m_flywheels.tuneInit();
    }

    public void testPeriodic() {
        boolean enabled = SmartDashboard.getBoolean("Tune Enable", false);
        if (enabled) {
            //m_flywheels.tunePeriodic();
            m_feeder.tunePeriodic();
        } else {
            m_feeder.setPWM(-Deadband.linearScaledDeadband(
                    m_jack.getY(GenericHID.Hand.kRight), Constants.OI.XBOX_DEADBAND));
        }
    }


}
