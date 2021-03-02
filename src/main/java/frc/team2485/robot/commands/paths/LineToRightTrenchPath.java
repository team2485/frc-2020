package frc.team2485.robot.commands.paths;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.*;
import frc.team2485.WarlordsLib.Limelight;
import frc.team2485.robot.Constants;
import frc.team2485.robot.subsystems.Drivetrain;

import java.util.ArrayList;

public class LineToRightTrenchPath extends SequentialCommandGroup {

    Drivetrain m_drivetrain;

    public LineToRightTrenchPath(Drivetrain drivetrain, Limelight limelight) {
        super();

        addCommands(this.getRamseteCommand(drivetrain, limelight), new RunCommand(()-> m_drivetrain.driveVolts(0,0)));
    }

    private RamseteCommand getRamseteCommand (Drivetrain driveTrain, Limelight limelight) {

        double tx = limelight.getTargetHorizontalOffset(0);
        double startPointY = -1 * (Constants.Autonomous.POWER_PORT_X_POS + (Constants.Autonomous.INITIATION_LINE_X * Math.tan( Math.toRadians(tx)))); //check sign of tx
        double endPointY = Constants.Autonomous.RIGHT_TRENCH_END_Y;
        double startPointX = Constants.Autonomous.INITIATION_LINE_X;

        //voltage constant to ensure we don't accelerate too fast

        DifferentialDriveVoltageConstraint autoVoltageConstraint = Constants.Autonomous.AUTO_VOLTAGE_CONSTRAINT;

        //create config for trajectory

        TrajectoryConfig trajectoryConfig = Constants.Autonomous.TRAJECTORY_CONFIG;

        //initial + (90% of delta)k

        Rotation2d submissionRotation = new Rotation2d(Math.PI);
        Pose2d submissionAutoStart = new Pose2d(3.19, startPointY, submissionRotation); //maybe maybe it zero at least to start
        Pose2d submissionAutoEnd = new Pose2d(7.62, -0.88, submissionRotation);
        Translation2d submissionAutoTranslation = new Translation2d(6.192, -0.88); //change names
        ArrayList<Translation2d> submissionAutoWaypoints = new ArrayList<Translation2d>();
        submissionAutoWaypoints.add(submissionAutoTranslation);

        Trajectory submissionAuto = TrajectoryGenerator.generateTrajectory(submissionAutoStart, submissionAutoWaypoints, submissionAutoEnd, trajectoryConfig);

        RamseteCommand submissionAutoRamseteCommand = new RamseteCommand(
                submissionAuto, //giving the trajectory itself
                driveTrain::getPose, //method reference to the method that returns pose
                new RamseteController(Constants.ArtemisTerms.K_RAMSETE_B, Constants.ArtemisTerms.K_RAMSETE_ZETA), //the ramsete controller object
                new SimpleMotorFeedforward(Constants.ArtemisTerms.KS_VOLTS, //drive feedforward
                        Constants.ArtemisTerms.KV_VOLT_SECONDS_PER_METER,
                        Constants.ArtemisTerms.KA_VOLT_SECONDS_SQUARED_PER_METER),
                Constants.ArtemisTerms.K_DRIVE_KINEMATICS, //drive kinematics used to convert chassis speeds to wheel speeds
                driveTrain::getWheelSpeeds, //method reference to the method that returns wheel speeds
                new PIDController(Constants.ArtemisTerms.KP_DRIVE_VEL, 0, 0), //left side pid controller
                new PIDController(Constants.ArtemisTerms.KP_DRIVE_VEL, 0, 0), //right side pid controller
                //Ramsete command passes volts to the callback
                driveTrain::driveVolts, //method reference to the method that passed voltage outputs to the motors
                driveTrain //passing the robot drive itself
        );

        return submissionAutoRamseteCommand;
    }

}
