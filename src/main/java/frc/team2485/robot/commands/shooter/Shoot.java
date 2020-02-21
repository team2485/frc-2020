package frc.team2485.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.team2485.WarlordsLib.Limelight;
import frc.team2485.robot.Constants;
import frc.team2485.robot.subsystems.Flywheels;
import frc.team2485.robot.subsystems.Hood;

import java.util.function.DoubleSupplier;

public class Shoot extends ParallelCommandGroup {


    public Shoot(Flywheels flywheels, Hood hood, Limelight limelight, DoubleSupplier finalYVelocity) {
        super();
        double vfy = finalYVelocity.getAsDouble();
        double ty = limelight.getTargetVerticalOffset(Constants.Robot.LIMELIGHT_TY_DEFAULT_VALUE); //gets vertical angle from limelight
        double xDist = getX(ty, Constants.Robot.HEIGHT_FROM_LL_TO_PORT); //finds x distance (horizontal) to port
        double v0y = getv0y(vfy, Constants.Robot.HEIGHT_FROM_LL_TO_PORT, Constants.Shooter.GRAVITY_ACCELERATION_CONSTANT); //finds initial y velocity based on final y velocity and height changes
        double timeOfTrajectory = gettimeOfTraj(v0y, vfy, Constants.Shooter.GRAVITY_ACCELERATION_CONSTANT, Constants.Robot.HEIGHT_FROM_SHOOTER_TO_PORT); //finds time of trajectory based on y velocities, distances, and accelerations
        double vfx = getvfX(xDist, timeOfTrajectory); //finds final x velocity from time of trajectory and distance traversed
        double v0x = getv0xFromVfx(timeOfTrajectory, vfx, Constants.PowerCell.POWER_CELL_DRAG_COEFF, Constants.PowerCell.POWER_CELL_MASS); //finds initial x velocity using drag
        //double thetaApproach = getThetaApproach(vfx, vfy); //finds approach angle to port using final x and y velocities

        double thetaLaunch = getThetaLaunch(v0x, v0y); //finds launch angle using initial component velocities
        double RPM = getRPM(v0x, thetaLaunch, Constants.PowerCell.POWER_CELL_RADIUS, Constants.Shooter.RPM_CONVERSION_FACTOR); //finds launch RPM using initial angle+velocity

        this.addCommands(new SetFlywheels(flywheels, RPM * Constants.Shooter.FLYWHEEL_ENERGY_LOSS_FACTOR), new SetHood(hood, thetaLaunch));

    }

    private static double getX(double ty, double LLtoPort){
        return LLtoPort / Math.tan(Math.toRadians(ty));
    }

    private static double getv0y(double vfy, double LLtoPort, double g){
        double twogy = 2 * g * LLtoPort;
        return Math.sqrt((vfy * vfy) + twogy); //what if vfy is negative
    }

    private static double gettimeOfTraj(double v0y, double vfy, double shooterToPort, double g) {
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
    private static double getvfX(double x, double t){
        return x / t;
    }

    private static double getv0xFromVfx(double timeOfTraj, double vfx, double dragCoeff, double mass){
        return vfx / (Math.pow(Math.E, (-dragCoeff * timeOfTraj) / mass));
    }

    public static double getThetaApproach(double vfx, double vfy){
        return Math.atan( vfy / vfx); //radians
    }

    public static double getThetaLaunch( double v0x, double v0y){
        return Math.atan(v0y/v0x);
    }

    public static double getRPM(double v0x, double thetaLaunch, double radius, double rpmConversionFactor){
        double numerator = Math.sqrt( (v0x *v0x) + ((v0x * Math.tan(thetaLaunch)) * (v0x * Math.tan(thetaLaunch))));
        double denominator = radius * rpmConversionFactor;
        return numerator / denominator;
    }


}
