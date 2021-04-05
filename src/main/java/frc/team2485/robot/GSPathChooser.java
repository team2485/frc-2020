package frc.team2485.robot;

import frc.team2485.robot.subsystems.*;
import edu.wpi.first.wpilibj.MedianFilter;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import frc.team2485.WarlordsLib.Limelight;
import frc.team2485.WarlordsLib.robotConfigs.SavableConfigs;
import frc.team2485.WarlordsLib.robotConfigs.LoadableConfigs;
import frc.team2485.WarlordsLib.robotConfigs.RobotConfigs;
import frc.team2485.WarlordsLib.robotConfigs.Configurable;
import frc.team2485.robot.commands.*;
import frc.team2485.robot.commands.paths.*;

public class GSPathChooser{

    private Drivetrain m_drivetrain;
    private Limelight m_limelight;
    private LowMagazine m_lowMagazine;
    private MedianFilter m_llFilter;

    private static enum PATH_OPTIONS {
        ARED, ABLUE, BRED, BBLUE, NONE;

         Command getPath(Drivetrain drivetrain) {
            switch(this) {
                case ARED:
                    return new ARedPath(drivetrain);
                case ABLUE:
                    return new ABluePath(drivetrain);
                case BRED:
                    return new BRedPath(drivetrain);
                case BBLUE:
                    return new BBluePath(drivetrain);
                default:
                    return null;
            }
        } 

    }

    private PATH_OPTIONS m_layout;

    public GSPathChooser(Drivetrain drivetrain, LowMagazine lowMagazine, Limelight limelight){
        m_drivetrain = drivetrain;
        m_limelight = limelight;
        m_lowMagazine = lowMagazine;
        m_llFilter = new MedianFilter(10);
        m_layout = PATH_OPTIONS.NONE;
                
        this.addToShuffleboard();
    }

    public void addToShuffleboard() {
        ShuffleboardTab tab = Shuffleboard.getTab("Autonomous");
        tab.addString("Color Evaluation", ()-> {return getEval().toString();});
        tab.addNumber("LL area", ()->{return m_llFilter.calculate(m_limelight.getTargetArea(0));});
        //tab.addBoolean("Recieved color evaluation from configs", ()->m_configsLoaded.getBoolean("isRed", this.getEval()));
        tab.addNumber("Limelight angle", ()->{return m_limelight.getTargetHorizontalOffset(0);});
    }

    private void evaluate() {
        double area = m_llFilter.calculate(m_limelight.getTargetArea(0));
        if (area < 1) {
            m_layout = PATH_OPTIONS.NONE;
        } else if(area < 2.3) {
            m_layout = PATH_OPTIONS.BBLUE;
        } else if (area < 3.2) {
            m_layout = PATH_OPTIONS.ABLUE;
        } else if (area < 4) {
            m_layout = PATH_OPTIONS.BRED;
        } else {
            m_layout = PATH_OPTIONS.ARED;
        }
    }

    public Command getPath() {
        Command path;
        this.evaluate();
        return m_layout.getPath(m_drivetrain);
    }

    public void setEval(PATH_OPTIONS layout) {
        m_layout = layout;
    }

    public PATH_OPTIONS getEval() {
        return m_layout;
    }


    
}