package frc.team2485.robot.lib;

import com.ctre.phoenix.motorcontrol.SensorCollection;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import frc.team2485.WarlordsLib.sensors.EncoderWrapper;
import frc.team2485.WarlordsLib.sensors.TalonSRXEncoder;

public class ArmabotTurretEncoder extends SensorCollection implements EncoderWrapper {

    private static final int PULSES_PER_REVOLUTION = 4096;

    private double distancePerRevolution = 1;

    public ArmabotTurretEncoder(TalonSRX motorController) {
        super(motorController);
    }

    @Override
    public double getPosition() {
        return this.distancePerRevolution * ((this.getPulseWidthRiseToFallUs() - 1024) / 8) / PULSES_PER_REVOLUTION;
    }

    /**
     * Reset position of encoder
     *
     * @param position position to set encoder
     */
    @Override
    public void resetPosition(double position) {
        this.setPulseWidthPosition((int) (position * PULSES_PER_REVOLUTION / this.distancePerRevolution), 0);
    }

    /**
     * Set distance per revolution scale factor for encoder
     *
     * @param distance distance per rev for encoder
     */
    @Override
    public void setDistancePerRevolution(double distance) {
        this.distancePerRevolution = distance;
    }

    /**
     * Get velocity of encoder scaled by setDistancePerRevolution
     *
     * @return velocity of encoder
     */
    @Override
    public double getVelocity() {
        return this.getPulseWidthVelocity();
    }
}
