package frc.team2485.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SendableRegistry;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team2485.WarlordsLib.control.WL_PIDController;
import frc.team2485.WarlordsLib.robotConfigs.RobotConfigs;
import frc.team2485.robot.Constants;
import frc.team2485.robot.subsystems.LowMagazine;

public class IncrementLowMagazine extends CommandBase {

    private WL_PIDController m_positionController;
    private LowMagazine m_lowMagazine;
    private int m_numIncrements;

    public IncrementLowMagazine(LowMagazine lowMagazine, int numIncrements) {
        addRequirements(lowMagazine);
        this.m_lowMagazine = lowMagazine;
        this.m_positionController = new WL_PIDController();
        this.m_numIncrements = numIncrements;

        SendableRegistry.add(m_positionController, "Low Magazine Position Controller");
        RobotConfigs.getInstance().addConfigurable("lowMagazinePositionController", m_positionController);
    }
    public IncrementLowMagazine(LowMagazine lowMagazine) {
        addRequirements(lowMagazine);
        this.m_lowMagazine = lowMagazine;
        this.m_positionController = new WL_PIDController();
        this.m_numIncrements = 1;

        SendableRegistry.add(m_positionController, "Low Magazine Position Controller");
        RobotConfigs.getInstance().addConfigurable("lowMagazinePositionController", m_positionController);
    }


    public void initialize() {
        m_positionController.setSetpoint(m_numIncrements * Constants.Magazine.INDEX_BY_ONE_POS);
    }

    public void execute() {
        m_lowMagazine.setCurrent(m_positionController.calculate(m_lowMagazine.getEncoderPosition()));
    }

    public boolean isFinished() {
        return m_positionController.atSetpoint();
    }

}
