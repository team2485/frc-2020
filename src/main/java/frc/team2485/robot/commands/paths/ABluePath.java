package frc.team2485.robot.commands.paths;

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
import java.nio.file.Path;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import java.io.IOException;

public class ABluePath extends SequentialCommandGroup {

    public ABluePath (Drivetrain drivetrain) {
        super();
        this.addCommands(this.getRamseteCommand(drivetrain), new RunCommand(()-> drivetrain.driveVolts(0,0)));
    }

    private RamseteCommand getRamseteCommand(Drivetrain drivetrain) {
        String trajectoryJSON = "output/ablue.wpilib.json";
        Trajectory trajectory = null;
        try {
            Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
            trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
        } catch (IOException ex) {
            DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, ex.getStackTrace());
        }

        drivetrain.resetOdometry(trajectory.getInitialPose());
        RamseteCommand ramsete = new RamseteCommand(
            trajectory, drivetrain::getPose,
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

        return ramsete;
    }
}