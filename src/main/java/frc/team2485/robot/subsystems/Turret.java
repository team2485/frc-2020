package frc.team2485.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.wpilibj.smartdashboard.SendableRegistry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpiutil.math.MathUtil;
import frc.team2485.WarlordsLib.Limelight;
import frc.team2485.WarlordsLib.control.WL_PIDController;
import frc.team2485.WarlordsLib.motorcontrol.PIDTalonSRX;
import frc.team2485.WarlordsLib.robotConfigs.RobotConfigs;
import frc.team2485.WarlordsLib.sensors.TalonSRXEncoder;
import frc.team2485.robot.Constants;
import frc.team2485.robot.lib.ArmabotEncoderWrapper;

public class Turret extends SubsystemBase {

    private PIDTalonSRX m_talon;

    private Limelight m_limelight;

    private TalonSRXEncoder m_encoder;

    private WL_PIDController anglePID;

    private double minAngle, maxAngle, bufferZoneSize;

    private double m_encoderOffset;

    public Turret() {

        m_talon = new PIDTalonSRX(Constants.Turret.TALON_PORT, ControlMode.MotionProfile);
        m_talon.enableVoltageCompensation();

        m_encoder = new TalonSRXEncoder(m_talon, TalonSRXEncoder.TalonSRXEncoderType.ABSOLUTE, 4096);
        m_encoder.setDistancePerRevolution(360);

        anglePID = new WL_PIDController();
        anglePID.setTolerance(10);

        m_limelight = new Limelight();

        minAngle = Constants.Turret.MIN_POSITION;
        maxAngle = Constants.Turret.MAX_POSITION;

        bufferZoneSize = Constants.Turret.BUFFER_ZONE_SIZE;

        RobotConfigs.getInstance().addConfigurable("Turret encoder", m_encoder);
        RobotConfigs.getInstance().addConfigurable("Turret PID", anglePID);
//        RobotConfigs.getInstance().addConfigurable("Turret TalonSRX PID", m_talon);

        SendableRegistry.add(anglePID, "Turret PID");
//        SendableRegistry.add(m_talon, "Turret Talon");

        SmartDashboard.putData(this);
    }

    public void setPWM(double pwm) {
        double output = 0;
        if (pwm < 0) {
            output = Math.copySign(MathUtil.clamp(Math.abs(pwm), 0, (minAngle -  this.getEncoderPosition()) * (-1 / bufferZoneSize)), pwm);
        } else if (pwm > 0) {
            output = MathUtil.clamp(pwm, 0, (maxAngle - this.getEncoderPosition()) * (1 / bufferZoneSize));
        }

        m_talon.set(output);
    }

    /**
     * Reset the encoder position to a given position
     * @param position position to set to.
     */
    public void setEncoderPosition(double position) {
        m_encoder.resetPosition(position);
        m_encoderOffset = position - m_encoder.getPosition() ;
        System.out.println(m_encoderOffset);
    }

    public boolean usePositionPID(double setpoint) {
        this.setPWM(anglePID.calculate(this.getEncoderPosition(), MathUtil.clamp(setpoint, minAngle, maxAngle)));
        return anglePID.atSetpoint();
    }

    public boolean positionPIDAtSetpoint() {
        return anglePID.atSetpoint();
    }

    /**
     * Get turret encoder position
     * @return encoder position in radians
     */
    public double getEncoderPosition() {
        return m_encoder.getPosition() + m_encoderOffset;
    }

    public Limelight getLimelight() {
        return this.m_limelight;
    }

    public double getMinAngle() {
        return this.minAngle;
    }

    public double getMaxAngle() {
        return this.maxAngle;
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Turret Encoder Position", (this.getEncoderPosition()));
    }
}
