package frc.team2485.robot;

import frc.team2485.robot.subsystems.*;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command;
import frc.team2485.WarlordsLib.Limelight;
import frc.team2485.WarlordsLib.robotConfigs.SavableConfigs;
import frc.team2485.WarlordsLib.robotConfigs.LoadableConfigs;
import frc.team2485.WarlordsLib.robotConfigs.RobotConfigs;
import frc.team2485.WarlordsLib.robotConfigs.Configurable;
import frc.team2485.robot.commands.paths.*;

public class GSPathChooser implements Configurable{

    private Drivetrain m_drivetrain;
    private Limelight m_limelight;
    private boolean m_isRed; //if false, is blue
    private boolean m_isA; //if false, is B path
    private SendableChooser<Boolean> m_pathChooser; //A or B

    public GSPathChooser(Drivetrain drivetrain, Limelight limelight){
        m_drivetrain = drivetrain;
        m_limelight = limelight;
        this.evaluate();
        
        m_pathChooser = new SendableChooser<Boolean> ();
        m_pathChooser.addOption("A", Boolean.valueOf(true));
        m_pathChooser.addOption("B", Boolean.valueOf(false));
        m_pathChooser.setDefaultOption("A", Boolean.valueOf(true));
        m_isA = ((Boolean) m_pathChooser.getSelected()).booleanValue();
        RobotConfigs.getInstance().addConfigurable("GSPathChooser", this);

        this.addToShuffleboard();
    }

    public void addToShuffleboard() {
        ShuffleboardTab tab = Shuffleboard.getTab("Autonomous");
        tab.add("Path A or B?", m_pathChooser);
        tab.addBoolean("Color Evaluation", this::getEval);
        tab.addNumber("Limelight angle", ()->m_limelight.getTargetHorizontalOffset(0));
    }

    public void evaluate() {
        if(m_limelight.getTargetHorizontalOffset(0) > Constants.Autonomous.COLOR_CUTOFF_LL_ANGLE) {
            m_isRed = false;
            System.out.println("not red");
        } else {
            m_isRed = true;
            System.out.println("red");
        }

    }

    public Command getPath() {
        this.evaluate();
        Command path;

        if(m_isA) {
            if(m_isRed) {
                path = new ARedPath(m_drivetrain);
            } else {
                path = new ABluePath(m_drivetrain);
            }

        } else {
            if (m_isRed) {
                path = new BRedPath(m_drivetrain);
            } else {
                path = new BBluePath(m_drivetrain);
            }
        }

        return path;
    }

    public void setEval(boolean isRed) {
        m_isRed = isRed;
    }

    public boolean getEval() {
        return m_isRed;
    }

    public void loadConfigs(LoadableConfigs configs) {
        this.setEval(configs.getBoolean("isRed", this.getEval()));
    }

    public void saveConfigs(SavableConfigs configs) {
        configs.put("isRed", this.getEval());
        m_isA = ((Boolean) m_pathChooser.getSelected()).booleanValue();
    }

    
}