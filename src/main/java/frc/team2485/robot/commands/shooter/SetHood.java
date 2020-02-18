package frc.team2485.robot.commands.shooter;

import edu.wpi.first.wpilibj.smartdashboard.SendableRegistry;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team2485.WarlordsLib.control.WL_PIDController;
import frc.team2485.WarlordsLib.robotConfigs.RobotConfigs;
import frc.team2485.robot.subsystems.shooter.Hood;

public class SetHood extends CommandBase {
    private Hood m_hood;
    private WL_PIDController m_pidController;
    private double m_angle;

    public SetHood(Hood hood, double angle) {
        addRequirements(hood);
        this.m_hood = hood;
        this.m_angle = angle;
        this.m_pidController = new WL_PIDController();
        SendableRegistry.add(m_pidController, "Hood Position Controller");
        RobotConfigs.getInstance().addConfigurable("hoodPositionController", m_pidController);
    }

    public void initialize() {
        m_pidController.setSetpoint(m_angle);
    }

    public void execute() {
        m_hood.setCurrent(m_pidController.calculate(m_hood.getEncoderPosition()));
    }

    public boolean isFinished() {
        return m_pidController.atSetpoint();
    }
}
