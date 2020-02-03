/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team2485.robot;

public final class Constants {

    public static final String CONFIGS_FILE = "/home/lvuser/constants.csv";

    public static final boolean TUNING_MODE = true;

    /**
     * Constants specific to operator interface
     */
    public static final class OI {

        /**
         * Jack's XBOX port
         */
        public static final int JACK_PORT = 0;

        /**
         * We know it's not actually Suraj this year but whatever. It's a Suraj.
         */
        public static final int SURAJ_PORT = 1;

        /**
         * Deadband threshold for all Xbox controllers.
         */
        public static final double XBOX_DEADBAND = 0.2;
    }

    public static final class Drivetrain {

        public static final int SPARK_LEFT_PORT_MASTER = 10;
        public static final int SPARK_LEFT_PORT_SLAVE_2 = 11;
        public static final int SPARK_LEFT_PORT_SLAVE_3 = 12;

        public static final int SPARK_RIGHT_PORT_MASTER = 14;
        public static final int SPARK_RIGHT_PORT_SLAVE_2 = 15;
        public static final int SPARK_RIGHT_PORT_SLAVE_3 = 16;

        public static final int SPARK_LEFT_ENCODER = 10;
        public static final int SPARK_RIGHT_ENCODER = 14;

        public static final int ENCODER_CPR = 250;

        public static final double WHEEL_RADIUS = 3;

        public static final int CURRENT_LIMIT = 18;
    }

    public static final class Turret {

        public static final int TALON_PORT = 1;

        /**
         * counts per revolution of the encoder
         */
        public static final int ENCODER_CPR = 4096;

        public static final double TURRET_SPEED = 360.0 * 0.02; // degrees per ~20 milliseconds

        public static final double MIN_POSITION = -175; // degrees
        public static final double MAX_POSITION = 175; // degrees
        public static final double BUFFER_ZONE_SIZE = 10; // degrees

        public static final double TURRET_PID_TOLERANCE = 2; // degrees
    }
}
