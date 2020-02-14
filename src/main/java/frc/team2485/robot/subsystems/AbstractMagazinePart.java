package frc.team2485.robot.subsystems;

public interface AbstractMagazinePart {

    boolean setPosition(double position);

    double getEncoderPosition();

    boolean atPositionSetpoint();

}
