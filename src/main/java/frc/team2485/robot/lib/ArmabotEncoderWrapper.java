package frc.team2485.robot.lib;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import frc.team2485.WarlordsLib.robotConfigs.Configurable;
import frc.team2485.WarlordsLib.robotConfigs.LoadableConfigs;
import frc.team2485.WarlordsLib.robotConfigs.SavableConfigs;
import frc.team2485.WarlordsLib.sensors.TalonSRXEncoder;

public class ArmabotEncoderWrapper extends TalonSRXEncoder implements Configurable {

    private double m_offset;

    public ArmabotEncoderWrapper(TalonSRX motorController) {
        super(motorController, TalonSRXEncoderType.ABSOLUTE, 4096);
    }

    @Override
    public void resetPosition(double position) {
        this.m_offset = position + this.getPosition() + m_offset;
    }

    @Override
    public double getPosition() {
        double output =  (this.getPulseWidthPosition() / (double) this.getPulsesPerRevolution());

        while (output <= 0) {
            output ++;
        }

        while (output >= 1) {
            output --;
        }

        output *= this.getDistancePerRevolution();
        output -= m_offset;


        return output;
//        return (this.getDistancePerRevolution() * ((this.getPulsesPerRevolution() + 1024) / 8.0) / this.getPulsesPerRevolution()) - m_offset;
    }

    @Override
    public void loadConfigs(LoadableConfigs configs) {
        this.m_offset = configs.getDouble("encoder m_offset", this.m_offset);
    }

    @Override
    public void saveConfigs(SavableConfigs configs) {
        configs.put("encoder m_offset", this.m_offset);
    }
}
