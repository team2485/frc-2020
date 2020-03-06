package frc.team2485.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team2485.robot.subsystems.Drivetrain;

public class SetDrivePWM extends CommandBase {
    private Drivetrain drivetrain;
    private double pwm;
    private double timeout;
    private double time = 0;

    public SetDrivePWM(Drivetrain drivetrain, double pwm, double timeout){
        this.drivetrain = drivetrain;
        this.pwm = pwm;
        this.timeout = timeout;
    }

    public void execute(){
      drivetrain.setPWM(pwm);
      time++;

    }

    @Override
    public boolean isFinished() {
        return time >= timeout;
    }
}
