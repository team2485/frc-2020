import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.team2485.WarlordsLib.Limelight;
import frc.team2485.robot.Constants;
import frc.team2485.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj.util.Units;
import java.util.ArrayList;

public class ABluePath extends SequentialCommandGroup {

    public ABluePath (double startX, double startY, Drivetrain drivetrain) {
        super();
        this.addCommands(this.getRamseteCommand(drivetrain, startX, startY), new RunCommand(()-> drivetrain.driveVolts(0,0)));
    }

    private RamseteCommand getRamseteCommand(Drivetrain drivetrain, double startX, double startY) {
        var autoVolageConstraint = Constants.Autonomous.AUTO_VOLTAGE_CONSTRAINT;
        TrajectoryConfig config = Constants.Autonomous.TRAJECTORY_CONFIG;
        
        Pose2d A_BlueStart = new Pose2d(Units.feetToMeters(startX), Units.feetToMeters(startY), new Rotation2d(0));
        //End coordinates: X = middle of endzone, Y = y coordinate of last inner waypoint
        Pose2d A_BlueEnd = new Pose2d(Units.feetToMeters(28.75), Units.feetToMeters(7.5), new Rotation2d(0));
        Translation2d A_Blue1 = new Translation2d(Units.feetToMeters(15), Units.feetToMeters(2.5));
        Translation2d A_Blue2 = new Translation2d(Units.feetToMeters(17.5), Units.feetToMeters(10));
        Translation2d A_Blue3 = new Translation2d(Units.feetToMeters(22.5), Units.feetToMeters(7.5));
        ArrayList <Translation2d> A_BlueWaypoints = new ArrayList<Translation2d>();
        A_BlueWaypoints.add(A_Blue1);
        A_BlueWaypoints.add(A_Blue2);
        A_BlueWaypoints.add(A_Blue3);

        Trajectory A_BlueTrajectory = TrajectoryGenerator.generateTrajectory(A_BlueStart, A_BlueWaypoints, A_BlueEnd, config);

        RamseteCommand A_BlueRamsete = new RamseteCommand(
            A_BlueTrajectory, drivetrain::getPose,
            new RamseteController(Constants.ArtemisTerms.K_RAMSETE_B, Constants.ArtemisTerms.K_RAMSETE_ZETA),
            new SimpleMotorFeedforward(Constants.ArtemisTerms.KS_VOLTS,
                                    Constants.ArtemisTerms.KV_VOLT_SECONDS_PER_METER,
                                    Constants.ArtemisTerms.KA_VOLT_SECONDS_SQUARED_PER_METER),
            Constants.ArtemisTerms.K_DRIVE_KINEMATICS,
            drivetrain::getWheelSpeeds,
            new PIDController(Constants.ArtemisTerms.KP_DRIVE_VEL, 0, 0),
            new PIDController(Constants.ArtemisTerms.KP_DRIVE_VEL, 0, 0),
            //Ramsete command passes volts to the callback
            drivetrain::driveVolts,
            drivetrain
        );

        return A_BlueRamsete;
    }
}