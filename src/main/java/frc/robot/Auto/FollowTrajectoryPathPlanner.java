package frc.robot.Auto;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
//import frc.robot.Constants.PathPlannerConstants;
import frc.robot.subsystems.SwerveSubsystem;

public class FollowTrajectoryPathPlanner extends CommandBase {

  private SwerveSubsystem driveSubsystem;
  private String pathName;
  private boolean zeroInitialPose;

  PPSwerveControllerCommand followTrajectoryPathPlannerCommand;
  private boolean done = false;

  /** Creates a new FollowTrajectoryPathPlanner. */
  public FollowTrajectoryPathPlanner(SwerveSubsystem driveSubsystem, String pathName, boolean zeroInitialPose) {
    this.driveSubsystem = driveSubsystem;
    addRequirements(driveSubsystem);
    
    this.pathName = pathName;
    this.zeroInitialPose = zeroInitialPose;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Makes a trajectory                                                     
    PathPlannerTrajectory trajectoryToFollow = PathPlanner.loadPath(pathName,1,1, false);

    // Resets the pose of the robot if true (should generally only be true for the first path of an auto)
    if (zeroInitialPose) {
      driveSubsystem.resetOdometry(trajectoryToFollow.getInitialHolonomicPose());
    }

    // PID controllers
    PIDController xController = new PIDController(0.4, 0, 0);
    PIDController yController = new PIDController(0.4, 0, 0);
    PIDController thetaController = new PIDController(3, 0, 0);

  //   ProfiledPIDController thetaController = new ProfiledPIDController(
  //   AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
  // thetaController.enableContinuousInput(-Math.PI, Math.PI);

    // Create a PPSwerveControllerCommand. This is almost identical to WPILib's SwerveControllerCommand, but it uses the holonomic rotation from the PathPlannerTrajectory to control the robot's rotation.
    //driveSubsystem.resetOdometry(trajectoryToFollow.getInitialPose());
    followTrajectoryPathPlannerCommand = new PPSwerveControllerCommand(
      trajectoryToFollow,
      driveSubsystem::getPose, // Functional interface to feed supplier
      DriveConstants.kDriveKinematics,
      xController,
      yController,
      thetaController,
      driveSubsystem::setModuleStates,
      false,
      driveSubsystem
    );
    
    followTrajectoryPathPlannerCommand.schedule();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    done = followTrajectoryPathPlannerCommand.isFinished();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return done;
  }
}