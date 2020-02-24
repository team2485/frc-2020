package frc.team2485.robot.commands;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableRegistry;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team2485.WarlordsLib.Tunable;
import frc.team2485.WarlordsLib.control.WL_PIDController;
import frc.team2485.WarlordsLib.robotConfigs.RobotConfigs;
import frc.team2485.robot.Constants;
import frc.team2485.robot.subsystems.Hood;

import java.util.function.DoubleSupplier;

public class SetHood extends CommandBase implements Tunable {

    private Hood m_hood;
    private WL_PIDController m_controller;
    private DoubleSupplier m_angle;
    private boolean m_finishWhenAtTarget;

    public SetHood(Hood hood, double angle) {
        this(hood, () -> angle, true);
    }

    public SetHood(Hood hood, DoubleSupplier angle) {
        this(hood,  angle, true);
    }

    public SetHood(Hood hood, DoubleSupplier angle, boolean finishWhenAtTarget) {
        addRequirements(hood);

        this.m_hood = hood;
        this.m_angle = angle;
        this.m_controller = new WL_PIDController();
        this.m_finishWhenAtTarget = finishWhenAtTarget;

        RobotConfigs.getInstance().addConfigurable(Constants.Shooter.HOOD_POSITION_CONTROLLER_CONFIGURABLE_LABEL, m_controller);

        this.addToShuffleboard();
    }

    public void addToShuffleboard() {
        SendableRegistry.add(m_controller, "Hood Position Controller");
        ShuffleboardTab tab = Shuffleboard.getTab(Constants.Shooter.TAB_NAME);
        tab.add(this);
        tab.add(m_controller);
    }

    @Override
    public void execute() {
        m_controller.setSetpoint(m_angle.getAsDouble());
        m_hood.setPWM(m_controller.calculate(m_hood.getEncoderPosition()));
    }

    @Override
    public boolean isFinished() {
        if (m_finishWhenAtTarget) {
            return m_controller.atSetpoint();
        }
        return false;
    }

    @Override
    public void tunePeriodic() {
        m_hood.setPWM(m_controller.calculate(m_hood.getEncoderPosition()));
    }
}
