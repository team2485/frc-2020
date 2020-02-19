/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team2485.robot;

public final class Constants {

    public static final String CONFIGS_FILE = "/home/lvuser/constants.csv";

    /**
     * Constants specific to operator interface
     */
    public static final class OI {

        /**
         * Jack's XBOX port
         */
        public static final int JACK_PORT = 0;

        /**
         * Suraj's XBOX port
         */
        public static final int SURAJ_PORT = 0;



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

    public static final class Magazine {

        public static final int TALON_LOW_MAX_CURRENT = 60;
        public static final int TALON_HIGH_MAX_CURRENT = 60;

        //replace ports with real values
        public static final int TALON_LOW_PORT = 22;
        public static final int TALON_HIGH_PORT = 23;

        public static final int ENTRANCE_IR_PORT = 1;
        public static final int TRANSFER_IR_PORT = 2;
        public static final int EXIT_IR_PORT = 3;

        public static final int HIGH_MAGAZINE_BALL_CAPACITY = 4;

        public static final double ENCODER_VELOCITY_DEADBAND = 0.1;

        //Replace below with actual number
        public static final double LOW_INTAKE_BY_ONE_POS = 0.7;
        public static final double HIGH_INDEX_BY_ONE_POS = 0.7;

        //replace below with actual number
        public static final double LOW_BELT_PWM = 0.2;
        public static final double FAST_INTAKE_PWM = 0.5;
        public static final double NORMAL_BALL_INCREMENT_TIMEOUT = 0.2;
    }
}
