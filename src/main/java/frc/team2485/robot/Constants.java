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

    public static final double NOMINAL_VOLTAGE = 12;

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
         * Suraj's XBOX port
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

        public static final int LEFT_ENCODER_SPARK = 10;
        public static final int RIGHT_ENCODER_SPARK = 14;

        public static final int MAX_CURRENT = 18;

        public static final int PIGEON_IMU_PORT = 1;

        public static final int ENCODER_CPR = 250 * 4; // 4x encoding

        public static final double WHEEL_RADIUS = 3; // inches

        public static final double UP_RAMP_RATE = 0.4;
        public static final double DOWN_RAMP_RATE = 0.1; //pwm deltas

        public static final String RESET_GYRO_LABEL = "Zero Gyro";
    }

    public static final class IntakeArm {

        public static final int MAX_CURRENT = 2; //amps

        public static final int TALON_PORT = 20;

        public static final int ENCODER_DIO_PORT = 0;

        public static final double ENCODER_PULSES_PER_REVOLUTION = 174.9;

        public static final double TOP_POSITION_DEGREES = 0;

        public static final double BOTTOM_POSITION_DEGREES = 90; //temp

        public static final double SPEED = 0.7;
    }

    public static final class Magazine {

        public static final int SPARK_LOW_MAX_CURRENT = 60;
        public static final int SPARK_HIGH_MAX_CURRENT = 60;

        //replace ports with real values
        public static final int SPARK_LOW_PORT = 22;
        public static final int SPARK_HIGH_PORT = 23;

        public static final int ENTRANCE_IR_PORT = 2;
        public static final int TRANSFER_IR_PORT = 3;
        public static final int EXIT_IR_PORT = 4;

        public static final int HIGH_MAGAZINE_BALL_CAPACITY = 4;

        public static final double ROLLER_RADIUS = 1.3 / 2;

        public static final double HIGH_GEAR_RATIO = 1;
//        public static final double HIGH_GEAR_RATIO = 18.0/24;
        public static final double LOW_GEAR_RATIO = 12.0/30;

        public static final double LOW_INTAKE_BY_ONE_POS = -7;
        public static final double HIGH_INDEX_BY_ONE_POS = -6;

        //replace below with actual number
        public static final double LOW_BELT_PWM = -0.7;
        public static final double FAST_INTAKE_PWM = -0.5;
        public static final double OUTTAKE_PWM = 0.5;
        public static final double NORMAL_BALL_INCREMENT_TIMEOUT = 0.5;

        public static final double HIGH_MAGAZINE_POSITION_CONTROLLER_THRESHOLD = 1;

        public static final double COUNTER_MAX_PERIOD = 0.01;
        public static final int SAMPLES_TO_AVERAGE = 40;

        public static final String TAB_NAME = "Magazine";

        public static final String HIGH_MAGAZINE_POSITION_CONTROLLER_CONFIGURABLE_LABEL = "highMagazineSparkPositionController";

        public static final String LOW_MAGAZINE_VELOCITY_CONTROLLER_CONFIGURABLE_LABEL = "lowMagazineSparkVelocityController";
    }

    public static final class Feeder {
        public static final int SPARK_PORT = 33;
        public static final int MAX_CURRENT = 80; //keep this
        public static final int SPARK_FEEDER_MAX_STALL_CURRENT = 60; //keep this

        public static final String VELOCITY_CONTROLLER_CONFIGURABLE_LABEL = "feederSpark";

        public static final double OUTTAKE_PWM = 0.5;
    }

    public static final class Shooter {
        public static final int SPARK_HOOD_PORT = 32;
        public static final double SPARK_HOOD_MAX_CURRENT = 0;

        public static final int HOOD_ENCODER_CPR = 1000 * 4; //4x encoding
        public static final double HOOD_DISTANCE_PER_REVOLUTION = 360;

        // these are relative to the vertical axis
        public static final double HOOD_BOTTOM_POSITION_DEG = 74;
        public static final double HOOD_TOP_POSITION_DEG = 45;

        //both in radians relative to horizontal
        //check if these should be in degrees or radians
        public static final double HOOD_MAX_THETA = Math.toRadians(90 - HOOD_TOP_POSITION_DEG);
        public static final double HOOD_MIN_THETA = Math.toRadians(90 - HOOD_BOTTOM_POSITION_DEG);


        public static final int SPARK_FLYWHEEL_LEFT_PORT = 30;
        public static final double SPARK_FLYWHEEL_LEFT_MAX_CURRENT = 0;

        public static final int SPARK_FLYWHEEL_RIGHT_PORT = 31;
        public static final double SPARK_FLYWHEEL_RIGHT_MAX_CURRENT = 0;

        public static final double RPM_CONVERSION_FACTOR = 0.10472;
        public static final double FLYWHEEL_ENERGY_LOSS_FACTOR = 1;
        public static final double GRAVITY_ACCELERATION_CONSTANT = 9.8; //meters per second
        public static final double LIMELIGHT_ANGLE_FROM_HORIZONTAL = 14.34; //degrees

        public static final double RPM_ADJUST = 500;
        public static final double HOOD_ADJUST = 10;

        public static final String TAB_NAME = "Shooter";


        public static final String LEFT_FLYWHEEL_VELOCITY_CONTROLLER_CONFIGURABLE_LABEL = "flywheelsLeftSparkVelocityController";
        public static final String RIGHT_FLYWHEEL_VELOCITY_CONTROLLER_CONFIGURABLE_LABEL = "flywheelsLeftSparkVelocityController";
        public static final String HOOD_POSITION_CONTROLLER_CONFIGURABLE_LABEL = "hoodPositionController";
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

    public static final class Turret {

        public static final boolean TUNING_MODE = false;

        public static final int TALON_PORT = 25;

        public static final double TURRET_ADJUST = 10;

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
        public static final double BUFFER_ZONE_SIZE = 50; // degrees

        public static final double TURRET_PID_TOLERANCE = 2; // degrees

        public static final String TAB_NAME = "Turret";
        public static final String POSITION_CONTROLLER_CONFIGURABLE_LABEL = "Turret TalonSRX PID";
        public static final String ZERO_TURRET_LABEL = "Zero Turret";
    }

    public static final class Climber {
        public static final int TALON_PORT = 18;
    }
}
