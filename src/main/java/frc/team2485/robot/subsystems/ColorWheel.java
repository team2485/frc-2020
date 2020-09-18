package frc.team2485.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team2485.WarlordsLib.motorcontrol.WL_SparkMax;
import edu.wpi.first.wpilibj.DriverStation;

public class ColorWheel extends SubsystemBase {
    private ColorSensorV3 m_colorSensor;
    private WL_SparkMax m_sparkMax;

    private class RGB {
        public int r, g, b;

        public RGB(int r, int g, int b) {
            this.r = r;
            this.g = g;
            this.b = b;
        }

        public RGB(int cyan, int magenta, int yellow, int black) {

        }

        public int getRed() {
            return r;
        }

        public int getGreen() {
            return g;
        }

        public int getBlue() {
            return b;
        }
    }

    public ColorWheel() {

        m_colorSensor = new ColorSensorV3();
        m_sparkMax = new WL_SparkMax(Constants.ColorWheel.MOTOR_PORT);

    }

    // set pwm
    public void set(double pwm) {
        m_sparkMax.set(pwm);
    }

    // get color to spin to
    public RGB getColorToSpinTo() {
        String gameData;
        gameData = DriverStation.getInstance().getGameSpecificMessage();
        if (gameData.length() > 0) {
            switch (gameData.charAt(0)) {
                case 'B':
                    // Blue case code
                    return new RGB(100, 0, 0, 0);
                case 'G':
                    // Green case code
                    return new RGB(100, 0, 0, 0);
                case 'R':
                    // Red case code
                    return new RGB(100, 0, 0, 0);
                case 'Y':
                    // Yellow case code
                    return new RGB(100, 0, 0, 0);
                default:
                    // currupt
                    return getColorToSpinTo();
            }
        } else {
            // Code for no data received yet
        }

    }

    // get rgb values from sensor
    public int getRed() {
        return m_colorSensor.getRed();
    }

    public int getGreen() {
        return m_colorSensor.getGreen();
    }

    public int getBlue() {
        return m_colorSensor.getBlue();
    }

    // rgb to cmyk
    public double convertToCyan(int r) {
        double rc = r / 255;

        return (1 - rc) / (1);

    }

    public double convertToMagenta(int g) {
        double gc = g / 255;

        return (1 - gc) / (1);

    }

    public double convertToYellow(int b) {
        double bc = b / 255;

        return (1 - bc) / (1);

    }
    // cmyk to color

}
