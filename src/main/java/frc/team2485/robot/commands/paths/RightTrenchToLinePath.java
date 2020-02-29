package frc.team2485.robot.commands.paths;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.team2485.WarlordsLib.Limelight;
import frc.team2485.robot.Constants;
import frc.team2485.robot.subsystems.Drivetrain;

import java.util.ArrayList;

//THIS IS A DUPLICATE OF THE LINE TO RIGHT TRENCH PATH. DO NOT USE UNTIL EDITED WITH CORRECT PATH VALUES -MR
public class RightTrenchToLinePath extends SequentialCommandGroup {

    Drivetrain m_drivetrain;

    public RightTrenchToLinePath(Drivetrain drivetrain, Limelight limelight) {
        super();

        addCommands(this.getRamseteCommand(limelight), new RunCommand(()-> m_drivetrain.driveVolts(0,0)));
    }

    private RamseteCommand getRamseteCommand (Limelight limelight) {
        double tx = limelight.getTargetHorizontalOffset(0);
        double startPointY = -1 * (Constants.Autonomous.POWER_PORT_X_POS + (Constants.Autonomous.INITIATION_LINE_X * Math.tan( Math.toRadians(tx)))); //check sign of tx
        double endPointY = Constants.Autonomous.RIGHT_TRENCH_END_Y;
        double startPointX = Constants.Autonomous.INITIATION_LINE_X;

        //initial + (90% of delta)k

        Rotation2d zeroRotation = new Rotation2d(0);
        Pose2d rightTrenchStart = new Pose2d(3.169, startPointY, zeroRotation);
        //Pose2d threeBallStart = new Pose2d(startPointX, 3.169, zeroRotation);
        Pose2d rightTrenchEnd = new Pose2d(8.106, -0.71, zeroRotation);
        //Pose2d threeBallEnd = new Pose2d(0.71, 8.106, zeroRotation);
        Translation2d middleTranslation = new Translation2d(5.768, startPointY + (Constants.ApolloTerms.K_PATH_Y * (endPointY - startPointY)));
        //Translation2d middleTranslation = new Translation2d(startPointX + (Constants.kPathY * (endPointX - startPointX)), 5.768);
        ArrayList<Translation2d> rightTrenchInteriorWaypoints = new ArrayList<>();
        rightTrenchInteriorWaypoints.add(middleTranslation);


        Trajectory rightTrenchTrajectory = TrajectoryGenerator.generateTrajectory(rightTrenchStart, rightTrenchInteriorWaypoints, rightTrenchEnd, Constants.Autonomous.TRAJECTORY_CONFIG);

        RamseteCommand rightTrenchPath = new RamseteCommand(
                rightTrenchTrajectory, //giving the trajectory itself
                m_drivetrain::getPose, //method reference to the method that returns pose
                new RamseteController(Constants.ApolloTerms.K_RAMSETE_B, Constants.ApolloTerms.K_RAMSETE_ZETA), //the ramsete controller object
                new SimpleMotorFeedforward(Constants.ApolloTerms.KS_VOLTS, //drive feedforward
                        Constants.ApolloTerms.KV_VOLT_SECONDS_PER_METER,
                        Constants.ApolloTerms.KA_VOLT_SECONDS_SQUARED_PER_METER),
                Constants.ApolloTerms.K_DRIVE_KINEMATICS, //drive kinematics used to convert chassis speeds to wheel speeds
                m_drivetrain::getWheelSpeeds, //method reference to the method that returns wheel speeds
                new PIDController(Constants.ApolloTerms.KP_DRIVE_VEL, 0, 0), //left side pid controller
                new PIDController(Constants.ApolloTerms.KP_DRIVE_VEL, 0, 0), //right side pid controller
                //Ramsete command passes volts to the callback
                m_drivetrain::driveVolts, //method reference to the method that passed voltage outputs to the motors
                m_drivetrain //passing the robot drive itself
        );

        return rightTrenchPath;
    }
}
