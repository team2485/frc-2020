package frc.team2485.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.team2485.WarlordsLib.Interpolator;
import frc.team2485.WarlordsLib.Limelight;
import frc.team2485.WarlordsLib.Point;
import frc.team2485.robot.Constants;
import frc.team2485.robot.subsystems.Flywheels;
import frc.team2485.robot.subsystems.Hood;

public class Autoset extends ParallelCommandGroup {
   

    public Autoset(Hood hood, Flywheels flywheels, Limelight limelight, Point[] rpmPoints, Point[] anglePoints) {
        super();
        double vOffset = limelight.getTargetVerticalOffset(0);
        Interpolator rpm = new Interpolator(rpmPoints);
        Interpolator angle = new Interpolator(anglePoints);
        addCommands(
            new SetHood(hood, angle.getOutput(vOffset)),
            new InstantCommand(()-> flywheels.setSetpoint(rpm.getOutput(vOffset)))
        );

    }
}
