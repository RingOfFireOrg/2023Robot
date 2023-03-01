package frc.robot.Auto;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.SwerveSubsystem;

import java.util.List;

public class AutoPath {
    private final SwerveSubsystem swerveSubsystem;
    private PathPlannerTrajectory trajectory;
    private PPSwerveControllerCommand swerveControllerCommand;
    private double startVel = 0.0;
    private double endVel = 0.0;
    private double maxVel = AutoConstants.kMaxSpeedMetersPerSecond;
    private double maxAccel = AutoConstants.kMaxAccelerationMetersPerSecondSquared;

    public AutoPath(SwerveSubsystem swerveSubsystem, String trajectoryFile) {
        this.swerveSubsystem = swerveSubsystem;
        this.trajectory = PathPlanner.loadPath(trajectoryFile, maxVel, maxAccel);

        generateAutoPathCommand();
    }

    public AutoPath(SwerveSubsystem swerveSubsystem, String trajectoryFile, double maxVel, double maxAccel) {
        this.swerveSubsystem = swerveSubsystem;
        this.maxVel = maxVel;
        this.maxAccel= maxAccel;
        this.trajectory = PathPlanner.loadPath(trajectoryFile, maxVel, maxAccel);

        generateAutoPathCommand();
    }

    private void generateAutoPathCommand() {
        swerveControllerCommand = new PPSwerveControllerCommand(
            trajectory, 
            swerveSubsystem::getPose,
            DriveConstants.kDriveKinematics, 
            swerveSubsystem.getxController(), 
            swerveSubsystem.getyController(), 
            swerveSubsystem.getThetaController(),
            swerveSubsystem::setModuleStates,
            swerveSubsystem);
    }

    /**
     * DO NOT use if it is a part of a command group
     * Use getAutoPath() instead

    */
    public Command initializeCmd() {
        return new ScheduleCommand(swerveControllerCommand.beforeStarting(
            new ParallelCommandGroup(
                new InstantCommand(() -> swerveSubsystem.resetOdometry(getInitPose())),
                new InstantCommand(() -> swerveSubsystem.zeroHeading()))
            )
        );
    }


    public Command getAutoPath() {
        return swerveControllerCommand;
    }

    public Command odometryReset() {
        return new InstantCommand(() -> swerveSubsystem.resetOdometry(getInitPose()));
    }

    public Command zeroHeading() {
        return new InstantCommand(() -> swerveSubsystem.zeroHeading());
    }

    public Command setBrake() {
        return new InstantCommand(() -> swerveSubsystem.brake(true));
    }

    public Command setCoast() {
        return new InstantCommand(() -> swerveSubsystem.brake(false));
    }

    public Pose2d getInitPose() {
        return trajectory.getInitialPose();
    }

    public Pose2d getEndPose() {
        return trajectory.getEndState().poseMeters;
    }
    
    public PathPlannerTrajectory getTrajectory() {
        return trajectory;
    }

    public double getPathDuration() {
        return trajectory.getTotalTimeSeconds();
    }
}