/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team2485.robot;

public final class Constants {

    public static final String CONFIGS_FILE = "/home/lvuser/constants.csv";

    public static final boolean TUNE_MODE = true;
    public static final String TUNE_ENABLE_LABEL = "Tune Enable";

    /**
     * Constants specific to operator interface
     */
    public static final class OI {

        /**
         * Jack's XBOX port
         */
        public static final int JACK_PORT = 0;

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

        public static final int LEFT_ENCODER_SPARK = 10;
        public static final int RIGHT_ENCODER_SPARK = 14;

        public static final int PIGEON_IMU_PORT = 1;

        public static final int ENCODER_CPR = 250 * 4; // 4x encoding

        public static final double WHEEL_RADIUS = 3; // inches

        public static final int MAX_CURRENT = 18;
    }
}
