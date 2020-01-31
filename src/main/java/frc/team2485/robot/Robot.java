/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team2485.robot;

import com.ctre.phoenix.sensors.PigeonIMU;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.team2485.WarlordsLib.robotConfigs.RobotConfigs;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class Robot extends TimedRobot {
    private Command m_autonomousCommand;

    private RobotContainer m_robotContainer;

    private Pose2d currentLocation;
    private Rotation2d currentRotation;

    //Using values from the limelight
    private NetworkTable limelight;
    private double tx;
    private double ty;

    //pigeon imu
    private PigeonIMU _pigeon;


    //using to calculate distance from pp
    private final double a1 = 15.5 * Math.PI / 180; //Angle of the camera to the ground currently a test value
    private final double deltaHeight = 80; //Limelight to the target, currently a test value
    private double visionDistance; // from the limelight to the base of the target


    //Origin to Power Port
    private final double OPP = 2743.0 / 12; //inches, equal to 228 7/12 inches as a mixed fraction
    private final double lenX = 629.25; //inches
    private final double lenY = 323.25; //inches
    private double calcX, calcY; //calculated location a the moment


    //angles of the robot and turret to the field
    private double robotHeading;
    private double turretHeading;
    //Angle difference from the robots heading and the turrets heading, negative if it turned left and positive if it turned right, can be >360 and <-360
    private double rAngle;


    //odometry
    private DifferentialDriveOdometry moOdometry; //Odometry class
    private double leftEncoderDistance;
    private double rightEncoderDistance;

    @Override
    public void robotInit() {
        RobotConfigs.getInstance().loadConfigsFromFile(Constants.CONFIGS_FILE);
        m_robotContainer = new RobotContainer();


        Rotation2d currentRotation = new Rotation2d(-Math.PI / 2); // aiming toward the opposing driver stations
        currentLocation = new Pose2d(154.875/*Inches*/, 0, currentRotation); //On the initiation line, but 0 on the y coord
        limelight = NetworkTableInstance.getDefault().getTable("limelight");

        // pigeon imu
        _pigeon = new PigeonIMU(0); /* example Pigeon with device ID 0 */

        //Starter values
        tx = 0; //Angle from the middle of the screen to the target x axis
        visionDistance = 0; // Total distance to the target
        ty = 0; //angle from the middle of the screen to the target y axis
        rAngle = 0;//Turret to Robot heading difference
        robotHeading = 0; //Gyro values
        turretHeading = 0;
        leftEncoderDistance = 0;
        rightEncoderDistance = 0;
        //reset all encoders too

        currentRotation = new Rotation2d(robotHeading); // making it the robotheading
        //Making robot odometry
        moOdometry = new DifferentialDriveOdometry(currentRotation, currentLocation);
    }

    @Override
    public void robotPeriodic() {
        robotHeading = _pigeon.getCompassHeading(); //set as a gyro value;
        currentRotation = new Rotation2d(robotHeading); // making it the robotheading
        //set the left and right encoder distances
        //set the turret heading

        if (limelight.getEntry("tv").getDouble(0) == 1) {

            tx = limelight.getEntry("tx").getDouble(0);
            ty = limelight.getEntry("ty").getDouble(0);
            visionDistance = deltaHeight / (Math.tan(a1 + ty));
            turretHeading = robotHeading + rAngle;
            turretHeading %= 2 * Math.PI;//one rotation

            if (turretHeading < Math.PI) {
                calcX = visionDistance * Math.sin(turretHeading + tx);
                calcY = OPP - visionDistance * Math.cos(turretHeading + tx);
            } else {
                calcX = lenX - visionDistance * Math.sin(turretHeading - Math.PI + tx);
                calcY = lenY - OPP + visionDistance * Math.cos(turretHeading - Math.PI + tx);
            }

            currentLocation = new Pose2d(calcX, calcY, currentRotation);

            //reset odometry with the new data
            moOdometry.resetPosition(currentLocation, currentRotation);

            //code to rest the gyro
            //Maybe use some lidar that is mounted on the front of the robot, fleshed out more on the localization Research Document
            //Maybe calculate the ratios of the sides of the bounding box around the target to reset the gyro, easiest with the robot being directly in front of the target
        } else {
            currentLocation = moOdometry.update(currentRotation, leftEncoderDistance/*use m_sparkLeftMaster*/, rightEncoderDistance);

        }

        SmartDashboard.putNumber("Vision Distance", visionDistance);
    }

    @Override
    public void disabledInit() {
        RobotConfigs.getInstance().saveConfigsToFile(Constants.CONFIGS_FILE);
    }

    @Override
    public void disabledPeriodic() {
    }

    @Override
    public void autonomousInit() {
        m_autonomousCommand = m_robotContainer.getAutonomousCommand();

        if (m_autonomousCommand != null) {
            m_autonomousCommand.schedule();
        }
    }

    @Override
    public void autonomousPeriodic() {
    }

    @Override
    public void teleopInit() {
        if (m_autonomousCommand != null) {
            m_autonomousCommand.cancel();
        }

        m_robotContainer.resetAll();
    }

    @Override
    public void teleopPeriodic() {
    }

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {
    }
}
