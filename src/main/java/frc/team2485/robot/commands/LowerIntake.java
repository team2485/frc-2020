package frc.team2485.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.team2485.robot.Constants;
import frc.team2485.robot.subsystems.LowMagazine;

public class LowerIntake extends SequentialCommandGroup {
    
    public LowerIntake(LowMagazine lowMagazine) {
        super(
            new InstantCommand(()-> lowMagazine.setPWM(Constants.Intake.LOWERING_PWM), lowMagazine),
            new WaitCommand(Constants.Intake.LOWERING_TIME),
            new InstantCommand(()-> lowMagazine.setPWM(0), lowMagazine)
        );
    }


}
