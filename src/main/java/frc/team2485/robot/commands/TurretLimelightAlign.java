package frc.team2485.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SendableRegistry;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team2485.WarlordsLib.Limelight;
import frc.team2485.WarlordsLib.control.WL_PIDController;
import frc.team2485.WarlordsLib.robotConfigs.RobotConfigs;
import frc.team2485.robot.subsystems.Turret;

public class TurretLimelightAlign extends CommandBase {

    private WL_PIDController m_controller;

    private Turret m_turret;

    /**
     * Aligns turret based on limelight.
     * @param turret
     */
    public TurretLimelightAlign(Turret turret) {
        addRequirements(turret);

        m_turret = turret;

        m_controller = new WL_PIDController();

        m_controller.setTolerance(1);

        SendableRegistry.add(m_controller, "Limelight Turret Controller");
        RobotConfigs.getInstance().addConfigurable("Limelight Turret Controller", m_controller);
    }

    @Override
    public void execute() {
        double output = -m_controller.calculate(m_turret.getLimelight().getTargetHorizontalOffset(0));
        m_turret.setPWM(output);
    }
}
