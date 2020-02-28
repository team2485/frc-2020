package frc.team2485.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team2485.WarlordsLib.motorcontrol.WL_TalonSRX;
import frc.team2485.robot.Constants;

public class Climber extends SubsystemBase {

    private WL_TalonSRX m_talon;

    public Climber() {
        m_talon = new WL_TalonSRX(Constants.Climber.TALON_PORT);
    }

    public void setPWM(double pwm) {
        m_talon.set(pwm);
    }

}
