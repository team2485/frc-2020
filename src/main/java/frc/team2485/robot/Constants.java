/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team2485.robot;

public final class Constants {

    public static final String CONFIGS_FILE = "/home/lvuser/constants.csv";

    public static final boolean DEBUG_MODE = true;

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

        public static final double SURAJ_LTRIGGER_THRESHOLD = 0.3;
        public static final double SURAJ_RTRIGGER_THRESHOLD = 0.3;
        /**
         * Deadband threshold for all Xbox controllers.
         */
        public static final double XBOX_DEADBAND = 0.2;
    }

    public static final class Drivetrain {

        public static final boolean TUNING_MODE = false;

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

    public static final class IntakeArm {

        public static final int MAX_CURRENT = 2; //amps

        public static final int TALON_PORT = 20;

        public static final int ENCODER_DIO_PORT = 0;

        public static final double ENCODER_PULSES_PER_REVOLUTION = 174.9;

        public static final double TOP_POSITION_DEGREES = 0;

        public static final double BOTTOM_POSITION_DEGREES = 90; //temp

        public static final double SPEED = 0.5;

    }

    public static final class Shooter {
        public static final int SPARK_HOOD_PORT = 32;
        public static final double SPARK_HOOD_MAX_CURRENT = 0;

        public static final int HOOD_ENCODER_CPR = 1000 * 4; //4x encoding
        public static final double HOOD_DISTANCE_PER_REVOLUTION = 360;

        // these are relative to the vertical axis
        public static final double HOOD_BOTTOM_POSITION = 74;
        public static final double HOOD_TOP_POSITION = 45;


        public static final int SPARK_FLYWHEEL_LEFT_PORT = 30;
        public static final double SPARK_FLYWHEEL_LEFT_MAX_CURRENT = 0;

        public static final int SPARK_FLYWHEEL_RIGHT_PORT = 31;
        public static final double SPARK_FLYWHEEL_RIGHT_MAX_CURRENT = 0;

        public static final int SPARK_FEEDER_PORT = 33;
        public static final int SPARK_FEEDER_MAX_CURRENT = 32;

        public static final double RPM_CONVERSION_FACTOR = 0.10472;
        public static final double FLYWHEEL_ENERGY_LOSS_FACTOR = 1;
        public static final double GRAVITY_ACCELERATION_CONSTANT = 9.8; //meters per second
    }

    public static final class PowerCell {
        public static final double POWER_CELL_MASS = 0.141748; //kg
        public static final double POWER_CELL_DRAG_COEFF = 0.116;
        public static final double POWER_CELL_RADIUS = 0.0508; //meters
    }

    public static final class Robot {
        public static final double HEIGHT_FROM_LL_TO_PORT = 1.58114915; //meters
        public static final double HEIGHT_FROM_SHOOTER_TO_PORT = 1.46685; //meters
        public static final double LIMELIGHT_TY_DEFAULT_VALUE = 15; //angle...change?

    }

    public static final class Magazine {

        public static final int TALON_LOW_MAX_CURRENT = 60;
        public static final int TALON_HIGH_MAX_CURRENT = 60;

        //replace ports with real values
        public static final int TALON_LOW_PORT = 22;
        public static final int TALON_HIGH_PORT = 23;

        public static final int ENTRANCE_IR_PORT = 2;
        public static final int TRANSFER_IR_PORT = 3;
        public static final int EXIT_IR_PORT = 4;

        public static final int HIGH_MAGAZINE_BALL_CAPACITY = 4;

        public static final double ENCODER_VELOCITY_DEADBAND = 0.1;

        public static final double ROLLER_DIAMETER = 0.5;

        //Replace below with actual number
        public static final double LOW_INTAKE_BY_ONE_POS = 9;
        public static final double HIGH_INDEX_BY_ONE_POS = 9;

        //replace below with actual number
        public static final double LOW_BELT_PWM = -0.2;
        public static final double FAST_INTAKE_PWM = -0.5;
        public static final double NORMAL_BALL_INCREMENT_TIMEOUT = 0.2;
    }

    public static final class Turret {

        public static final boolean TUNING_MODE = false;

        public static final int TALON_PORT = 25;

        /**
         * counts per revolution of the encoder
         */
        public static final int ENCODER_CPR = 4096;

        public static final double TURRET_SPEED = 360.0 * 0.02; // degrees per ~20 milliseconds

        public static final double MIN_POSITION = -135; // degrees
        public static final double MAX_POSITION = 170; // degrees

        /**
         * In manual mode the max pwm will linearly clamp starting at the buffer zone size before the min or max positions.
         */
        public static final double BUFFER_ZONE_SIZE = 10; // degrees

        public static final double TURRET_PID_TOLERANCE = 2; // degrees
    }
}
