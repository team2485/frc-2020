package frc.team2485.robot.commands;

import frc.team2485.robot.subsystems.ColorWheel;

public class SpinToColor extends CommandBase {

    private final ColorWheel m_colorWheel;

    public SpinToColor(ColorWheel colorWheel) {
        m_colorWheel = colorWheel;
    }

}
