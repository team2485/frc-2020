package frc.team2485.robot.commands;

import frc.team2485.robot.subsystems.Turret;

import java.util.function.DoubleSupplier;

public class TurretLimelightAlign extends TurretSetAngle {

    /**
     * Set angle based on Limelight
     *
     * @param turret
     */
    public TurretLimelightAlign(Turret turret) {
        super(turret, () -> turret.getEncoderPosition() + turret.getLimelight().getTargetHorizontalOffset(0));
    }

    public TurretLimelightAlign(Turret turret, DoubleSupplier adjust) {
        super(turret, () -> turret.getEncoderPosition() + turret.getLimelight().getTargetHorizontalOffset(0) + adjust.getAsDouble());
    }
}
