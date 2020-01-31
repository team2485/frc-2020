package frc.team2485.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.revrobotics.ControlType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team2485.WarlordsLib.motorcontrol.PIDSparkMax;
import frc.team2485.WarlordsLib.motorcontrol.PIDTalonSRX;
import frc.team2485.robot.Constants;

import java.util.function.BooleanSupplier;

public class Magazine extends SubsystemBase {
    private PIDTalonSRX m_talonLow;
    private PIDTalonSRX m_talonHigh;
    private PIDSparkMax m_sparkOuttake;

    //replace with correct logic for sensors returning whether ball passed -- based on testing
    private BooleanSupplier m_endBelt1; //end of belt one
    private BooleanSupplier m_startBelt2; //at start of belt two
    private BooleanSupplier m_endBelt2; //at...end of belt two!



    private int ballsContained;

    public Magazine() {
        m_talonLow = new PIDTalonSRX(Constants.Magazine.TALON_LOW_PORT, ControlMode.Position);
        m_talonHigh = new PIDTalonSRX(Constants.Magazine.TALON_HIGH_PORT, ControlMode.Position);
        m_sparkOuttake = new PIDSparkMax(Constants.Magazine.SPARK_OUTTAKE_PORT, ControlType.kVelocity);
        ballsContained = 0;

    }

    public void indexLowBelt() {
        m_talonLow.runPID(Constants.Magazine.INDEX_BY_ONE_POS);
    }

    public void indexHighBelt() {
        m_talonHigh.runPID(Constants.Magazine.INDEX_BY_ONE_POS);
    }

    public void runHighBelt() {
        m_talonLow.set(Constants.Magazine.TALON_HIGH_PWM);
    }

    public void stopHighBelt() {
        m_talonHigh.set(0);
    }

    public void runLowBelt() {
        m_talonLow.set(Constants.Magazine.TALON_LOW_PWM);
    }

    public void stopLowBelt() {
        m_talonLow.set(0);
    }

    public void runOuttakeWheels(double velocity) {
        m_sparkOuttake.runPID(velocity);
    }

    //called periodically
    public void updateBalls() {
        //draft logic, replace with tested logic
        if(m_endBelt1.getAsBoolean()) {
            ballsContained++;
        }
    }

    public int getBallsContained() {
        return ballsContained;
    }

    public void decreaseBallsContained() {
        ballsContained--;
    }

    public boolean getEndBelt1() {
        return m_endBelt1.getAsBoolean();
    }

    public boolean getStartBelt2() {
        return m_startBelt2.getAsBoolean();
    }

    public boolean getEndBelt2() {
        return m_endBelt2.getAsBoolean();
    }


}
