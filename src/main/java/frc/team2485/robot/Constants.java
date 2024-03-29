/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team2485.robot;

import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import frc.team2485.WarlordsLib.Point;

public final class Constants {

    public static final String CONFIGS_FILE = "/home/lvuser/constants.csv";

    public static final boolean DEBUG_MODE = true;

    public static final double NOMINAL_VOLTAGE = 12;

    public static final boolean TUNE_MODE = true;

    public static final String PID_ENABLE_LABEL = "PID Enable";

    public static final String TUNE_ENABLE_LABEL = "Tune Enable";

    public static final String TUNE_LAYER_LABEL = "Tune Layer";

    public static final String RESET_PID_LABEL = "Reset PID";

    public static final double GRAVITY_ACCELERATION_CONSTANT = 9.8; //meters per second

    /**
     * Constants specific to operator interface
     */
    public static final class OI {

        /**
         * Jack's XBOX port
         */
        public static final int SABRINA_PORT = 0;

        /**
         * Suraj's XBOX port
         */
        public static final int ALISHA_PORT = 1;

        public static final double SURAJ_LTRIGGER_THRESHOLD = 0.3;
        public static final double SURAJ_RTRIGGER_THRESHOLD = 0.3;

        /**
         * Deadband threshold for all Xbox controllers.
         */
        public static final double XBOX_DEADBAND = 0.1;
    }

    public static final class Robot {
        public static final double HEIGHT_FROM_LL_TO_PORT = 1.58114915 * 3.28084; //meters... well now it's feet
        public static final double HEIGHT_FROM_SHOOTER_TO_PORT = 1.46685; //meters
        public static final double LIMELIGHT_TY_DEFAULT_VALUE = 15; //angle...change?
        public static final double LIMELIGHT_ANGLE_FROM_HORIZONTAL = 14.34; //degrees
    }

    public static final class PowerCell {
        public static final double POWER_CELL_MASS = 0.141748; //kg
        public static final double POWER_CELL_DRAG_COEFF = 0.116;
        public static final double POWER_CELL_RADIUS = 0.0508; //meters
    }

    public static final class Setpoint {
        public final double LL_OFFSET_Y;
        public final double RPM;
        public final double ANGLE;

        public Setpoint(double offsetY, double rpm, double angle) {
            LL_OFFSET_Y = offsetY; 
            RPM = rpm;
            ANGLE = angle;
        }

        public Point toPointRPM () {
            return new Point(LL_OFFSET_Y, RPM);
        }

        public Point toPointAngle () {
            return new Point(LL_OFFSET_Y, ANGLE);
        }
    }

    public static final class Setpoints {
        public static final Setpoint INITIATION_LINE = new Setpoint(16, -4000, 27);
        public static final Setpoint CLOSE_TRENCH = new Setpoint(-0.6, -4500, 10);
        public static final Setpoint FAR = new Setpoint(-7, -4600, 10);

        public static Point[] getPointsRPM() {
            Point[] points = new Point[3];
            points[0] = INITIATION_LINE.toPointRPM();
            points[1] = CLOSE_TRENCH.toPointRPM();
            points[2] = FAR.toPointRPM();
            return points;
        }

        public static Point[] getPointsAngle() {
            Point[] points = new Point[3];
            points[0] = INITIATION_LINE.toPointAngle();
            points[1] = CLOSE_TRENCH.toPointAngle();
            points[2] = FAR.toPointAngle();
            return points;
        }

        // public static final Setpoint GREEN_ZONE = new Setpoint(-5000, 27);
        // public static final Setpoint YELLOW_ZONE = new Setpoint(-5000, 16.9);
        // public static final Setpoint BLUE_ZONE = new Setpoint(-5000, 11.25);
        // public static final Setpoint RED_ZONE = new Setpoint(-5000, 10);

    }

    public static final class Drivetrain {
        public static final int TALON_LEFT_PORT_LEADER = 12;
        public static final int TALON_LEFT_PORT_FOLLOWER_2 = 13;
        // public static final int TALON_LEFT_PORT_FOLLOWER_3 = 100;
    
        public static final int TALON_RIGHT_PORT_LEADER = 2;
        public static final int TALON_RIGHT_PORT_FOLLOWER_2 = 1;
        // public static final int TALON_RIGHT_PORT_FOLLOWER_3 = 100;
    
        public static final int LEFT_ENCODER_TALON = 12;
        public static final int RIGHT_ENCODER_TALON = 2;
    
        public static final int PIGEON_IMU_PORT = 0;
    
        public static final int MAX_CURRENT = 18;
    
        public static final int ENCODER_CPR = 250 * 4; // 4x encoding
    
        public static final double WHEEL_RADIUS = 0; // inches
    
        public static final double DISTANCE_PER_REVOLUTION = 2 * Math.PI * WHEEL_RADIUS;
    
        public static final double STEERING_SCALE = 0.8;
        public static final double THROTTLE_SCALE = 0.8;

        public static final double UP_RAMP_RATE = 0.4;
        public static final double DOWN_RAMP_RATE = 0.1;
    
        public static final String RESET_GYRO_LABEL = "Zero Gyro";
    
        public static final int CONTROLLER_PORT = 0;
    
    
      }
    
    public static final class IntakeArm {

        public static final int MAX_CURRENT = 2; //amps

        public static final int TALON_PORT = 18;

        public static final int ENCODER_DIO_PORT = 0;

        public static final double ENCODER_PULSES_PER_REVOLUTION = 174.9;

        public static final double TOP_POSITION_DEGREES = 0;

        public static final double BOTTOM_POSITION_DEGREES = 90; //temp

        public static final double UP_SPEED = 0.7;
        public static final double DOWN_SPEED = 0.7;


    }

    public static final class Magazine {

        public static final int SPARK_LOW_MAX_CURRENT = 80;
        public static final int SPARK_HIGH_MAX_CURRENT = 80;
        //replace ports with real values
        public static final int SPARK_LOW_PORT = 22;
        public static final int SPARK_HIGH_PORT = 23;

        public static final int ENTRANCE_IR_PORT = 2;
        public static final int TRANSFER_IR_PORT = 3;
        public static final int EXIT_IR_PORT = 4;

        public static final int HIGH_MAGAZINE_BALL_CAPACITY = 3;

        public static final double ROLLER_RADIUS = 1.3 / 2;

       public static final double HIGH_GEAR_RATIO = 1;


        public static final double LOW_GEAR_RATIO = 12.0/30;

        public static final double HIGH_DISTANCE_PER_REVOLUTION = HIGH_GEAR_RATIO * 2 * Math.PI * ROLLER_RADIUS;

        public static final double LOW_INTAKE_BY_ONE_POS = -7;
        public static final double HIGH_INDEX_BY_ONE_POS = -8.25;
        public static final double HIGH_INCREMENT_TELEOP = -5;
        public static final double PUSH_IN_INCREMENT = -6;

        //replace below with actual number
        public static final double LOW_BELT_INTAKE_PWM = -0.4;
        public static final double OUTTAKE_PWM = 0.2;
        public static final double NORMAL_BALL_INCREMENT_TIMEOUT = 1; //seconds

        public static final double HIGH_MAGAZINE_POSITION_CONTROLLER_THRESHOLD = 0.5;

        public static final double COUNTER_MAX_PERIOD = 0.01;
        public static final int SAMPLES_TO_AVERAGE = 40;

        public static final String TAB_NAME = "Magazine";

        public static final String HIGH_MAGAZINE_VELOCITY_CONTROLLER_CONFIGURABLE_LABEL = "highMagazineVelocityController";
        public static final String HIGH_MAGAZINE_POSITION_CONTROLLER_CONFIGURABLE_LABEL = "highMagazinePositionController";

        public static final String LOW_MAGAZINE_VELOCITY_CONTROLLER_CONFIGURABLE_LABEL = "lowMagazineVelocityController";

        public static final double MAGAZINE_MAX_VELOCITY = 120;
        public static final double MAGAZINE_MIN_VELOCITY = -120;

        public static final double LOW_INTAKE_VELOCITY = -70;

        

        // public static final int MAX_DEBOUNCE_TIME = 3;
    }

    public static final class Feeder {
        public static final int SPARK_PORT = 33;
        public static final int MAX_CURRENT = 80; //keep this
        public static final int SPARK_FEEDER_MAX_STALL_CURRENT = 60; //keep this

        public static final double GEAR_RATIO = 1.0 / 3;

        public static final double RADIUS = 1.4 / 2;

        public static final double DISTANCE_PER_REVOLUTION = GEAR_RATIO * 2 * Math.PI * RADIUS;

        public static final String VELOCITY_CONTROLLER_CONFIGURABLE_LABEL = "feederSpark";

        public static final double OUTTAKE_PWM = 0.5;
        public static final double INTAKE_PWM = -1;

        public static final double FEEDER_MAX_VELOCITY = 1000;
        public static final double FEEDER_MIN_VELOCITY = -1000;

        public static final String TAB_NAME = "Shooter";
    }

    public static final class Hood {
        public static final int SPARK_PORT = 32;
        public static final double SPARK_HOOD_MAX_CURRENT = 0;

        public static final int ENCODER_CPR = 1000 * 4; //4x encoding
        public static final double DISTANCE_PER_REVOLUTION = 360;

        public static final double HOOD_LEAD_SCREW_GEAR_RATIO = 1.0/5;

        // these are relative to the vertical axis
        public static final double HOOD_BOTTOM_POSITION_DEG = 10;
        public static final double HOOD_TOP_POSITION_DEG = 42;


        public static final String TAB_NAME = "Shooter";

        //both in radians relative to horizontal
        //check if these should be in degrees or radians
        public static final double MAX_THETA = Math.toRadians(HOOD_TOP_POSITION_DEG);
        public static final double MIN_THETA = Math.toRadians(HOOD_BOTTOM_POSITION_DEG);

        public static final double AUTO_HOOD_MANUAL_ADJUST = 0;

        public static final double MANUAL_ANGLE_SCALE = 20;

        public static final String HOOD_VELOCITY_CONTROLLER_CONFIGURABLE_LABEL = "hoodVelocityController";
        public static final String HOOD_POSITION_CONTROLLER_CONFIGURABLE_LABEL = "hoodPositionController";

        public static final double HOOD_MAX_VELOCITY = 2500;
        public static final double HOOD_MIN_VELOCITY = -2500;

        public static final double BUFFER_ZONE_SIZE = 6;

        public static final double HOOD_DEFAULT_INCREMENT = -10;
    }

    public static final class Flywheels {
        public static final int SPARK_FLYWHEEL_LEFT_PORT = 30;
        public static final int SPARK_FLYWHEEL_LEFT_MAX_CURRENT = 80;

        public static final int SPARK_FLYWHEEL_RIGHT_PORT = 31;
        public static final int SPARK_FLYWHEEL_RIGHT_MAX_CURRENT = 80;

        public static final double ARC_ADJUST = 5;

        public static final double RPM_CONVERSION_FACTOR = 0.10472;

        public static final double FLYWHEEL_ENERGY_LOSS_FACTOR = 0.9;

        public static final double FYWHEEL_OUTTAKE_PWM = 0.1;

        public static final double RPM_ADJUST = 0;

        public static final double GEAR_RATIO = 2/1;

        public static final double RPM_THRESHOLD = 75;

        public static final String FLYWHEELS_TAB_NAME = "Shooter";
        public static final String INDEXING_TAB_NAME = "Indexing";

        public static final String LEFT_VELOCITY_CONTROLLER_CONFIGURABLE_LABEL = "flywheelsLeftVelocityController";
        public static final String RIGHT_VELOCITY_CONTROLLER_CONFIGURABLE_LABEL = "flywheelsRightVelocityController";

        public static final double FLYWHEELS_MAX_VELOCITY = 5000;
        public static final double FLYWHEELS_MIN_VELOCITY = -5000;

        public static final double VELOCITY_ARM = 0.98; //temp
        public static final double VELOCITY_TRIGGER = 0.95; //temp

        //IR
        public static final int ENTRANCE_IR_PORT = 2;
        public static final int TRANSFER_IR_PORT = 0;
        public static final int EXIT_IR_PORT = 1;
        public static final int MAX_DEBOUNCE_TIME = 3;

    }

    public static final class Turret {

        public static final boolean TUNING_MODE = false;

        public static final int TALON_PORT = 25;

        public static final double AUTO_TURRET_MANUAL_ADJUST = 10;

        /**
         * counts per revolution of the encoder
         */
        public static final int ENCODER_CPR = 4096;

        public static final double TURRET_SPEED = 360.0 * 0.02; // degrees per ~20 milliseconds

        public static final double MIN_POSITION = -135; // degrees
        public static final double MAX_POSITION = 158; // degrees

        public static final double MAX_VELOCITY = 90; // degrees / second
        public static final double MIN_VELOCITY = -90; // degrees / second

        /**
         * In manual mode the max pwm will linearly clamp starting at the buffer zone size before the min or max positions.
         */
        public static final double BUFFER_ZONE_SIZE = 90; // degrees


        public static final double TURRET_PID_TOLERANCE = 1; // degrees

        public static final String TAB_NAME = "Turret";
        public static final String POSITION_CONTROLLER_CONFIGURABLE_LABEL = "turretPositionController";
        public static final String VELOCITY_CONTROLLER_CONFIGURABLE_LABEL = "turretVelocityController";
        public static final String ENCODER_OFFSET_CONFIGURABLE_LABEL = "turretEncoderOffset";
        public static final String ZERO_TURRET_LABEL = "Zero Turret";

        public static final double MANUAL_ANGLE_SCALE = 150;

		public static final int CURRENT_PIPELINE = 6;

		public static final double FINE_VELOCITY = 20;
    }

    public static final class Climber {
        public static final int TALON_PORT = 19;

        public static final double DEFAULT_PWM = -0.7;
    }

    public static final class Autonomous {
        public static final double INITIATION_LINE_X = 3.048;
        public static final double POWER_PORT_X_POS = 0.0254*94.66;
        public static final double RIGHT_TRENCH_END_Y = -0.71;

        public static final double PORT_ENTRANCE_Y_VELOCITY = 0;
        public static final double RPM_ADJUST = 0;
        public static final double HOOD_ADJUST = 0;

        public static final double SHOOTING_WARMUP_DELAY = 1;


        public static final DifferentialDriveVoltageConstraint AUTO_VOLTAGE_CONSTRAINT = new DifferentialDriveVoltageConstraint(
                new SimpleMotorFeedforward(ArtemisTerms.KS_VOLTS,
                        ArtemisTerms.KV_VOLT_SECONDS_PER_METER,
                        ArtemisTerms.KA_VOLT_SECONDS_SQUARED_PER_METER), ArtemisTerms.K_DRIVE_KINEMATICS, 10);


        //create config for trajectory

        public static final TrajectoryConfig TRAJECTORY_CONFIG = new TrajectoryConfig(0.5, 0.5)
                //kinematics to ensure max speed is obeyed + applying voltage constraint
                .setKinematics(ArtemisTerms.K_DRIVE_KINEMATICS).addConstraint(AUTO_VOLTAGE_CONSTRAINT)
                .setReversed(true);

        public static final TrajectoryConfig COMPLEX_TRAJECTORY_CONFIG = new TrajectoryConfig(1, 2)
                //kinematics to ensure max speed is obeyed + applying voltage constraint
                .setKinematics(ArtemisTerms.K_DRIVE_KINEMATICS).addConstraint(AUTO_VOLTAGE_CONSTRAINT);//sfw now


    }

    public static final class ArtemisTerms {
        //Constants for Artemis
        public static final double KS_VOLTS = 0.742;
        public static final double KV_VOLT_SECONDS_PER_METER = 1.86;
        public static final double KA_VOLT_SECONDS_SQUARED_PER_METER = 0.235;
        public static final double KP_DRIVE_VEL = 0.0000166;

        public static final double K_TRACK_WIDTH_METERS = 0.7946375975879432;

        public static final DifferentialDriveKinematics K_DRIVE_KINEMATICS = new DifferentialDriveKinematics(K_TRACK_WIDTH_METERS);

        public static final double K_MAX_SPEED_METERS_PER_SECOND = 1;
        public static final double K_MAX_ACCELERATION_METERS_PER_SECOND_SQUARE = 1;

        //public static final double kEncoderDistancePerPulse = (3 * 2 * Math.PI/250);

        public static final double K_ENCODER_DISTANCE_PER_ROTATIO = 2 * Math.PI * 0.0762;

        public static final double K_RAMSETE_B = 2;
        public static final double K_RAMSETE_ZETA = 0.7;

        public static final double K_PATH_Y = 0.9;

    }

    public static final class ApolloTerms {
        public static final double KS_VOLTS = 0.163;
        public static final double KV_VOLT_SECONDS_PER_METER = 2.62;
        public static final double KA_VOLT_SECONDS_SQUARED_PER_METER = 0.275;
        public static final double KP_DRIVE_VEL = 0.001;
        public static final double K_TRACK_WIDTH_METERS = 0.62;

        public static final DifferentialDriveKinematics K_DRIVE_KINEMATICS = new DifferentialDriveKinematics(K_TRACK_WIDTH_METERS);
        public static final double K_MAX_SPEED_METERS_PER_SECOND = 3;
        public static final double K_MAX_ACCELERATION_METERS_PER_SECOND_SQUARED = 3;

        //public static final double K_ENCODER_DISTANCE_PER_PULSE= (3 * 2 * Math.PI/250);

        public static final double K_ENCODER_DISTANCE_PER_ROTATION = 2 * Math.PI * 0.0762;

        public static final double K_RAMSETE_B = 2;
        public static final double K_RAMSETE_ZETA = 0.7;

        public static final double K_PATH_Y = 0.9;

    }

    public static final class Intake {
        
        public static final int SPARKX_PORT = 26; //temp
        public static final int SPARKZ_PORT = 27; //temp
    
        public static final double MAX_VELOCITY = 6000; //temp
        public static final double MIN_VELOCITY = -6000; //temp
        public static final double TOLERANCE = 200; //temp
    
        public static final String TAB_NAME = "Intake Rollers";

        public static final String X_VELOCITY_CONTROLLER_CONFIGURABLE_LABEL = "intakeXVelocityController"; 
        public static final String Z_VELOCITY_CONTROLLER_CONFIGURABLE_LABEL = "intakeZVelocityController"; 

        public static final double X_VELOCITY = -2500;
        public static final double Z_VELOCITY = -2500;

        public static final double LOWERING_PWM = 0.07;
        public static final double LOWERING_TIME = 10; //seconds

        public static final double OUTTAKE_PWM = 0.7;
      }

}
