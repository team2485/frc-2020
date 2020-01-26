package frc.team2485.robot.lib;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import frc.team2485.WarlordsLib.sensors.TalonSRXEncoder;

public class ArmabotTurretEncoder extends TalonSRXEncoder {

    public ArmabotTurretEncoder(TalonSRX motorController) {
        super(motorController, TalonSRXEncoderType.ABSOLUTE, 1024);
    }

    @Override
    public double getPosition() {
        return (this.getPulseWidthRiseToFallUs() - 1024) / 8;
    }
}
