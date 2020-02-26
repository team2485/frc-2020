package frc.team2485.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team2485.robot.subsystems.IntakeArm;

public class IntakeArmMove extends CommandBase {

    private final IntakeArm m_intakeArm;

    public enum IntakeArmPosition {
        TOP, BOTTOM
    }

    private final IntakeArmPosition m_position;
    private final double m_speed;

    public IntakeArmMove(IntakeArm intakeArm, IntakeArmPosition position, double speed) {
        super();

        this.m_intakeArm = intakeArm;
        this.m_position = position;
        this.m_speed = speed;

        addRequirements(intakeArm);
    }

    @Override
    public void execute() {
        int direction = this.m_position == IntakeArmPosition.TOP ? -1 : 1;
        this.m_intakeArm.setPWM(Math.copySign(this.m_speed, direction));
    }

    @Override
    public boolean isFinished() {
        return this.m_position == IntakeArmPosition.TOP ? m_intakeArm.getTopLimitSwitch() : m_intakeArm.getBottomLimitSwitch();
    }

    @Override
    public void end(boolean interrupted) {
        m_intakeArm.setPWM(0);
    }
}
