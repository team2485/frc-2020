package frc.team2485.robot.commands;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.team2485.WarlordsLib.Limelight;
import frc.team2485.robot.Constants;
import frc.team2485.robot.subsystems.Flywheels;
import frc.team2485.robot.subsystems.Hood;

import java.util.function.DoubleSupplier;

public class Shoot extends ParallelCommandGroup {

    private double m_hoodSetpoint;
    private double m_rpmSetpoint;
    private DoubleSupplier m_finalYVelocity, m_rpmAdjust, m_hoodAdjust;
    private Limelight m_limelight;
    private Hood m_hood;
    private Flywheels m_flywheels;
    private double initialVelocity;

    public Shoot(Flywheels flywheels, Hood hood, Limelight limelight, DoubleSupplier finalYVelocity, DoubleSupplier rpmAdjust, DoubleSupplier hoodAdjust) {
        super();
        this.m_flywheels = flywheels;
        this.m_hood = hood;
        this.m_limelight = limelight;
        this.m_finalYVelocity = finalYVelocity;
        this.m_rpmAdjust = rpmAdjust;
        this.m_hoodAdjust = hoodAdjust;
        this.m_hoodSetpoint = 0;
        this.m_rpmSetpoint = 0;
        this.addCommands(new SetFlywheels(flywheels, ()-> m_rpmSetpoint * Constants.Flywheels.FLYWHEEL_ENERGY_LOSS_FACTOR),
                new SetHood(hood, ()-> m_hoodSetpoint + hoodAdjust.getAsDouble()));
        this.addToShuffleboard();
        this.initialVelocity = 0; // for Widget
    }

    public void execute() {
        super.execute();
        double vfy = m_finalYVelocity.getAsDouble();
        double ty = m_limelight.getTargetVerticalOffset(Constants.Robot.LIMELIGHT_TY_DEFAULT_VALUE) + Constants.Robot.LIMELIGHT_ANGLE_FROM_HORIZONTAL; //gets vertical angle from m_limelight
        double xDist = getX(ty, Constants.Robot.HEIGHT_FROM_LL_TO_PORT); //finds x distance (horizontal) to port
        double v0y = getv0y(vfy, Constants.Robot.HEIGHT_FROM_LL_TO_PORT, Constants.GRAVITY_ACCELERATION_CONSTANT); //finds initial y velocity based on final y velocity and height changes
        double timeOfTrajectory = gettimeOfTraj(v0y, vfy, Constants.GRAVITY_ACCELERATION_CONSTANT, Constants.Robot.HEIGHT_FROM_SHOOTER_TO_PORT); //finds time of trajectory based on y velocities, distances, and accelerations
        double vfx = getvfX(xDist, timeOfTrajectory); //finds final x velocity from time of trajectory and distance traversed
        double v0x = getv0xFromVfx(timeOfTrajectory, vfx, Constants.PowerCell.POWER_CELL_DRAG_COEFF, Constants.PowerCell.POWER_CELL_MASS); //finds initial x velocity using drag

        //double thetaApproach = getThetaApproach(vfx, vfy); //finds approach angle to port using final x and y velocities

        double thetaLaunch = getThetaLaunch(v0x, v0y); //finds launch angle using initial component velocities


        m_hoodSetpoint = Math.toDegrees(getComplement(thetaLaunch))
                + m_hoodAdjust.getAsDouble(); //accounts for 90 degree shift
        m_rpmSetpoint = - getRPM(v0x, thetaLaunch, Constants.PowerCell.POWER_CELL_RADIUS, Constants.Flywheels.RPM_CONVERSION_FACTOR)
                + m_rpmAdjust.getAsDouble(); //finds launch RPM using initial angle+velocity

        this.addToShuffleboard();
    }
    
    public void addToShuffleboard() {
        ShuffleboardTab tab = Shuffleboard.getTab(Constants.Shooter.TAB_NAME);
        tab.addNumber(" Angle Setpoint", ()-> m_hoodSetpoint);
        tab.addNumber("RPM Setpoint", ()-> m_rpmSetpoint);

        //Data for the Shooter Widget: Initial Velocity and the Pitch of the shooter
        SmartDashboard.putNumber("Shooter/pitch",m_hood.getEncoderPosition());
        SmartDashboard.putNumber("Shooter/iv",initialVelocity);

    }
    public Hood getHood() {
        return m_hood;
    }
    
    public void setInitialVelocity(double newVal) {
        initialVelocity = newVal;
    }

    public static double getX(double ty, double LLtoPort){
        return LLtoPort / Math.tan(Math.toRadians(ty));
    }

    public static double getv0y(double vfy, double LLtoPort, double g){
        double twogy = 2 * g * LLtoPort;
        return Math.sqrt((vfy * vfy) + twogy); //what if vfy is negative
    }

    public static double gettimeOfTraj(double v0y, double vfy, double shooterToPort, double g) {
        double plusTime;
        double minusTime;
        double t1;
        double t2;
        plusTime = (v0y + Math.sqrt((v0y * v0y) - (4 * (g / 2) * shooterToPort))) / g;
        minusTime = (v0y - Math.sqrt((v0y * v0y) - (4 * (g / 2) * shooterToPort))) / g;

        if (plusTime <= minusTime) {
            t1 = plusTime;
            t2 = minusTime;
        } else {
            t1 = minusTime;
            t2 = plusTime;
        }

        if (vfy >= 0){
            return t1; //first one
        } else {
            return t2; //second one
        }
    }

    //changed to getvfx from get v0x --> check with Aditya
    public static double getvfX(double x, double t){
        return x / t;
    }

    public static double getv0xFromVfx(double timeOfTraj, double vfx, double dragCoeff, double mass){
        return vfx / (Math.pow(Math.E, (-dragCoeff * timeOfTraj) / mass));
    }

    public static double getThetaApproach(double vfx, double vfy){
        return Math.atan( vfy / vfx); //radians
    }

    public static double getThetaLaunch( double v0x, double v0y){
        double theta = Math.atan(v0y/v0x);
        if( theta > Constants.Hood.MAX_THETA){
            return Constants.Hood.MAX_THETA;
        } else if (theta < Constants.Hood.MIN_THETA){
            return Constants.Hood.MIN_THETA;
        } else {
            return theta;
        }
    }

    public static double getComplement(double theta){ return Math.PI/2 - theta; } //check if this should be Math.PI/2 or 90

    public static double getRPM(double v0x, double thetaLaunch, double radius, double rpmConversionFactor){
        double numerator = Math.sqrt( (v0x *v0x) + ((v0x * Math.tan(thetaLaunch)) * (v0x * Math.tan(thetaLaunch))));
        double denominator = radius * rpmConversionFactor;
        return numerator / denominator;
    }
}
