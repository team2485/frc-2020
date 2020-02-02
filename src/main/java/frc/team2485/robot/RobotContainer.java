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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
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
                new TurretLimelightAlign(m_turret)
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
                            SmartDashboard.putNumber("Pigeon heading", pigeon.getFusedHeading());
                            return - pigeon.getFusedHeading();
                        }
                )
        );

        // Seek
        m_suraj.getJoystickButton(XboxController.Button.kA).whenPressed(
                new SequentialCommandGroup(
                        new TurretSetAngle(m_turret, Constants.Turret.MIN_POSITION + Constants.Turret.BUFFER_ZONE_SIZE, true),
                        new TurretSetAngle(m_turret, Constants.Turret.MAX_POSITION - Constants.Turret.BUFFER_ZONE_SIZE, true)
                )
        );

        SmartDashboard.putData("Zero Turret", new InstantCommand(() ->
                m_turret.setEncoderPosition(0)
        ));

        SmartDashboard.putData("Zero Pigeon", new InstantCommand(() ->
                pigeon.setFusedHeading(0)
        ));
    }

//    public void resetAll() {
//        m_drivetrain.resetEncoders(0, 0);
//    }

    public Command getAutonomousCommand() {
        // temporary!
        m_autoCommand = new RunCommand(() -> {
        });

        return m_autoCommand;
    }
}
