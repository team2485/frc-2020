package frc.team2485.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team2485.WarlordsLib.motorcontrol.PIDTalonSRX;
import frc.team2485.robot.Constants;

import java.util.function.BooleanSupplier;

public class Magazine extends SubsystemBase {
    private PIDTalonSRX talonLow;
    private PIDTalonSRX talonHigh;

    //replace with correct logic for sensors returning whether ball passed -- based on testing
    private BooleanSupplier endBelt1; //end of belt one
    private BooleanSupplier startBelt2; //at start of belt two
    private BooleanSupplier endBelt2; //at...end of belt two!



    private int ballsContained;

    public Magazine() {
        talonLow = new PIDTalonSRX(Constants.Magazine.TALON_LOW_PORT, ControlMode.Position);
        talonHigh = new PIDTalonSRX(Constants.Magazine.TALON_HIGH_PORT, ControlMode.Position);
        ballsContained = 0;

    }

    public void indexLowBelt() {
        talonLow.runPID(Constants.Magazine.INDEX_BY_ONE_POS);
    }

    public void indexHighBelt() {
        talonHigh.runPID(Constants.Magazine.INDEX_BY_ONE_POS);
    }

    public void runHighBelt() {
        talonLow.set(Constants.Magazine.TALON_HIGH_PWM);
    }

    public void stopHighBelt() {
        talonHigh.set(0);
    }

    public void runLowBelt() {
        talonLow.set(Constants.Magazine.TALON_LOW_PWM);
    }

    public void stopLowBelt() {
        talonLow.set(0);
    }

    //called periodically
    public void updateBalls() {
        //draft logic, replace with tested logic
        if(endBelt1.getAsBoolean()) {
            ballsContained++;
        }
    }

    public int getBallsContained() {
        return ballsContained;
    }

    public boolean getEndBelt1() {
        return endBelt1.getAsBoolean();
    }

    public boolean getStartBelt2() {
        return startBelt2.getAsBoolean();
    }

    public boolean getEndBelt2() {
        return endBelt2.getAsBoolean();
    }


}
