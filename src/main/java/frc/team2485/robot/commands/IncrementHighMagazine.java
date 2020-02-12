package frc.team2485.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SendableRegistry;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team2485.WarlordsLib.control.WL_PIDController;
import frc.team2485.WarlordsLib.robotConfigs.RobotConfigs;
import frc.team2485.robot.Constants;
import frc.team2485.robot.subsystems.HighMagazine;

public class IncrementHighMagazine extends CommandBase {

    private WL_PIDController m_positionController;
    private HighMagazine m_highMagazine;
    private int m_numIncrements;

    public IncrementHighMagazine(HighMagazine highMagazine, int numIncrements) {
        addRequirements(highMagazine);
        this.m_highMagazine = highMagazine;
        this.m_positionController = new WL_PIDController();
        this.m_numIncrements = numIncrements;

        SendableRegistry.add(m_positionController, "High Magazine Position Controller");
        RobotConfigs.getInstance().addConfigurable("highMagazinePositionController", m_positionController);
    }
    public IncrementHighMagazine(HighMagazine highMagazine) {
        addRequirements(highMagazine);
        this.m_highMagazine = highMagazine;
        this.m_positionController = new WL_PIDController();
        this.m_numIncrements = 1;

        SendableRegistry.add(m_positionController, "High Magazine Position Controller");
        RobotConfigs.getInstance().addConfigurable("highMagazinePositionController", m_positionController);
    }


    public void initialize() {
        m_positionController.setSetpoint(m_numIncrements * Constants.Magazine.INDEX_BY_ONE_POS);
    }

    public void execute() {
        m_highMagazine.setCurrent(m_positionController.calculate(m_highMagazine.getEncoderPosition()));
    }

    public boolean isFinished() {
        return m_positionController.atSetpoint();
    }

}
