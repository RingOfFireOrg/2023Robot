package frc.robot.commands;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Auto.trajectoryTest;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.LimeLight;
import frc.robot.subsystems.SwerveSubsystem;

public class autoAdjust extends CommandBase {
  private final SwerveSubsystem s_swerve;
  private final LimeLight limelight3;
  public autoAdjust(SwerveSubsystem s_swerve, LimeLight limelight3) {
    this.s_swerve = s_swerve;
    this.limelight3 = limelight3;
    addRequirements(s_swerve);
    addRequirements(limelight3);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double[] crosshairVals = limelight3.getVisionVals();




    double xDiffrence  = crosshairVals[0];
    TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
      AutoConstants.kMaxSpeedMetersPerSecond,
      AutoConstants.kMaxAccelerationMetersPerSecondSquared)
      .setKinematics(DriveConstants.kDriveKinematics);

      Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
        new Pose2d(0, 0, new Rotation2d(0)), 
        List.of(
          new Translation2d(xDiffrence,0)
          
          ),
        new Pose2d(xDiffrence, 0, Rotation2d.fromDegrees(0)),
      trajectoryConfig);
        
      PIDController xController = new PIDController(AutoConstants.kPXController, 0, 0);
      PIDController yController = new PIDController(AutoConstants.kPXController, 0, 0);
      ProfiledPIDController thetaController = new ProfiledPIDController(
        AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
      thetaController.enableContinuousInput(-Math.PI,Math.PI);
    
      SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
        trajectory,
        s_swerve::getPose,
        DriveConstants.kDriveKinematics,
        xController,
        yController,
        thetaController,
        s_swerve::setModuleStates,
        s_swerve);
      new SequentialCommandGroup(
        new InstantCommand(() -> s_swerve.resetOdometry(trajectory.getInitialPose())),
        swerveControllerCommand,
        new InstantCommand(() -> s_swerve.stopModules()));      






  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
