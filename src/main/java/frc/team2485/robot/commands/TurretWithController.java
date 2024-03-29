package frc.team2485.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team2485.robot.subsystems.Turret;

import java.util.function.DoubleSupplier;
import frc.team2485.WarlordsLib.oi.WL_XboxController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Axis;
import frc.team2485.robot.Constants;
import frc.team2485.WarlordsLib.oi.Deadband;



public class TurretWithController extends CommandBase {
    private Turret m_turret;
    private WL_XboxController m_controller;
    public TurretWithController(Turret turret, WL_XboxController controller) {
        this.m_turret = turret;
        this.m_controller = controller;
        addRequirements(turret);
        }
        public void execute() {
            boolean rightPressed = m_controller.getPOV() > 0 && m_controller.getPOV() < 180;
            boolean leftPressed = m_controller.getPOV() > 180 && m_controller.getPOV() < 360;
            

            double adjust = 0;
            if(rightPressed) {
                adjust = Constants.Turret.FINE_VELOCITY;
            } else if (leftPressed) {
                adjust = -Constants.Turret.FINE_VELOCITY;
            }
            
            m_turret.runVelocityPID(
                getAxis(m_controller, Axis.kLeftX) * Constants.Turret.MAX_VELOCITY + adjust);
            
            
        }

        public boolean isFinished() {
            return false;
        }

        private double getAxis(WL_XboxController controller, Axis axis) {
            return Deadband.linearScaledDeadband(controller.getRawAxis(axis.value), Constants.OI.XBOX_DEADBAND);
        }
}