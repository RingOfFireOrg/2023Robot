package frc.robot;

import java.util.List;

import com.kauailabs.navx.frc.AHRS;

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
import frc.robot.commands.autoAdjust;
import frc.robot.commands.pistonIntakeGrab;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Auto.trajectoryTest;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.SwerveJoystickCommand;
import frc.robot.commands.armJoystickCommand;
import frc.robot.subsystems.LimeLight;
import frc.robot.subsystems.pistonIntake;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.linearSlideArm;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  public SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
  public LimeLight limeLightSubsystem = new LimeLight();
  public linearSlideArm armSubsystem = new linearSlideArm();
  public pistonIntake pistonIntakeSubsystem = new pistonIntake();
  


  private final XboxController driverController = new XboxController(OIConstants.kDriverControllerPort);
  private final XboxController operatorController = new XboxController(OIConstants.kOperatorControllerPort);

  public RobotContainer() {
    swerveSubsystem.setDefaultCommand(new SwerveJoystickCommand(
        swerveSubsystem,
        () -> -driverController.getRawAxis(OIConstants.kDriverYAxis),
        () -> driverController.getRawAxis(OIConstants.kDriverXAxis),
        () -> driverController.getRawAxis(OIConstants.kDriverRotAxis),
        () -> !driverController.getRawButton(OIConstants.kDriverFieldOrientedButtonIdx),
        () -> driverController.getRawButton(OIConstants.kAlignWithTargetButton),
        () -> driverController.getRawButton(OIConstants.kResetDirectionButton)
        ));
    armSubsystem.setDefaultCommand(new armJoystickCommand(
      armSubsystem, 
      () -> operatorController.getRawAxis(1), 
      () -> operatorController.getRawButton(4), 
      () -> operatorController.getRawButton(5)));

    pistonIntakeSubsystem.setDefaultCommand(new pistonIntakeGrab(
      pistonIntakeSubsystem
    ));
    configureButtonBindings();

  }

  private void configureButtonBindings() {
    //new JoystickButton(driverController, 2).whenPressed(() -> swerveSubsystem.zeroHeading());
    new JoystickButton(driverController, OIConstants.kAlignWithTargetButton).onTrue(new autoAdjust(swerveSubsystem,limeLightSubsystem));
    

  }









  

  
  public Command getAutonomousCommand() {
   // 1. Create trajectory settings
   TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
    AutoConstants.kMaxSpeedMetersPerSecond,
    AutoConstants.kMaxAccelerationMetersPerSecondSquared)
            .setKinematics(DriveConstants.kDriveKinematics);
// 2. Generate trajectory
final AHRS gyro = new AHRS();
gyro.reset();
Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
    new Pose2d(0, 0, new Rotation2d(0)),
    List.of(
            new Translation2d(.5, 0),
            new Translation2d(0, .5)
            ),
            new Pose2d(.4, .5, Rotation2d.fromDegrees(0)),
    
    trajectoryConfig);


// 3. Define PID controllers for tracking trajectory
PIDController xController = new PIDController(AutoConstants.kPXController, 0, 0);
PIDController yController = new PIDController(AutoConstants.kPYController, 0, 0);
ProfiledPIDController thetaController = new ProfiledPIDController(
    AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
thetaController.enableContinuousInput(-Math.PI, Math.PI);

// 4. Construct command to follow trajectory
SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
    trajectory,
    swerveSubsystem::getPose,
    DriveConstants.kDriveKinematics,
    xController,
    yController,
    thetaController,
    swerveSubsystem::setModuleStates,
    swerveSubsystem);

// 5. Add some init and wrap-up, and return everything
return new SequentialCommandGroup(
    new InstantCommand(() -> swerveSubsystem.resetOdometry(trajectory.getInitialPose())),
    swerveControllerCommand,
    new InstantCommand(() -> swerveSubsystem.stopModules()));
}
}

