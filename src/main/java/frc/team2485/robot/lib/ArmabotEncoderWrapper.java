package frc.team2485.robot.lib;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import frc.team2485.WarlordsLib.robotConfigs.Configurable;
import frc.team2485.WarlordsLib.robotConfigs.LoadableConfigs;
import frc.team2485.WarlordsLib.robotConfigs.SavableConfigs;
import frc.team2485.WarlordsLib.sensors.TalonSRXEncoder;

public class ArmabotEncoderWrapper extends TalonSRXEncoder implements Configurable {

    private double offset;

    public ArmabotEncoderWrapper(TalonSRX motorController) {
        super(motorController, TalonSRXEncoderType.ABSOLUTE, 4096);
    }

    @Override
    public void resetPosition(double position) {
        this.offset = this.getPosition() + offset;
    }

    @Override
    public double getPosition() {
        return (super.getPosition() - offset);
    }

    @Override
    public void loadConfigs(LoadableConfigs configs) {
        this.offset = configs.getDouble("encoder offset", this.offset);
    }

    @Override
    public void saveConfigs(SavableConfigs configs) {
        configs.put("encoder offset", this.offset);
    }
}
