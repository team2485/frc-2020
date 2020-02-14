package frc.team2485.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableRegistry;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team2485.WarlordsLib.control.WL_PIDController;
import frc.team2485.WarlordsLib.motorcontrol.PIDTalonSRX;
import frc.team2485.WarlordsLib.oi.Deadband;
import frc.team2485.WarlordsLib.robotConfigs.RobotConfigs;
import frc.team2485.robot.Constants;

import java.util.function.BooleanSupplier;

public class HighMagazine extends SubsystemBase implements AbstractMagazinePart {

    private PIDTalonSRX m_talon;

    private WL_PIDController m_positionController;

    private DigitalInput m_exitIR;

    private int m_numBalls;

    private BooleanSupplier m_transferIR;

    private boolean m_transferIRLastVal, m_exitIRLastVal;

    public HighMagazine(BooleanSupplier transferIR)  {
        m_talon = new PIDTalonSRX(Constants.Magazine.TALON_HIGH_PORT, ControlMode.Current);

        m_exitIR = new DigitalInput(Constants.Magazine.EXIT_IR_PORT);

        m_positionController = new WL_PIDController();

        m_numBalls = 0;

        m_transferIR = transferIR;

        SendableRegistry.add(m_positionController, "High Magazine Position Controller");
        RobotConfigs.getInstance().addConfigurable("highMagazinePositionController", m_positionController);


        ShuffleboardTab tab = Shuffleboard.getTab("Magazine");
        tab.addNumber("High Position", this::getEncoderPosition);
        tab.addNumber("High Velocity", this::getEncoderPosition);
        tab.addNumber("High Number of Balls", this::getNumBalls);
        tab.addBoolean("Exit IR", this::getTransferIR);
    }

    public void setPWM(double pwm) {
        m_talon.set(pwm);
    }

    public void setCurrent(double current) {
        m_talon.runPID(current);
    }

    @Override
    public boolean setPosition(double position) {
        m_talon.runPID(m_positionController.calculate(getEncoderPosition(), position));
        return m_positionController.atSetpoint();
    }

    @Override
    public boolean atPositionSetpoint() {
        return m_positionController.atSetpoint();
    }

    @Override
    public double getEncoderPosition() {
        return m_talon.getSensorCollection().getQuadraturePosition();
    }

    public double getEncoderVelocity() {
        return Deadband.deadband(m_talon.getSensorCollection().getQuadraturePosition(), Constants.Magazine.ENCODER_VELOCITY_DEADBAND);
    }

    public boolean getTransferIR() {
        return m_transferIR.getAsBoolean();
    }

    public boolean getExitIR() {
        return m_exitIR.get();
    }

    public int getNumBalls() {
        return m_numBalls;
    }

    @Override
    public void periodic() {

        if (!getTransferIR() && getTransferIR() != m_transferIRLastVal) {
            if (getEncoderVelocity() < 0) {
                m_numBalls++;
            } else if (getEncoderVelocity() > 0) {
                m_numBalls--;
            }
        }

        if (!getExitIR() && getExitIR() != m_exitIRLastVal) {
            if (getEncoderVelocity() < 0) {
                m_numBalls--;
            } else if (getEncoderVelocity() > 0 ) {
                m_numBalls++;
            }
        }

        m_exitIRLastVal = getExitIR();
        m_transferIRLastVal = getTransferIR();
    }
}
