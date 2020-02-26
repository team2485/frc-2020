package frc.team2485.robot.subsystems;

import frc.team2485.WarlordsLib.motorcontrol.WL_TalonSRX;
import frc.team2485.robot.Constants;

public class Climber {
    private WL_TalonSRX m_talon;

    public Climber() {
        m_talon = new WL_TalonSRX(Constants.Climber.TALON_PORT);
    }

    public void setPWM(double pwm) {
        m_talon.set(pwm);
    }
}
