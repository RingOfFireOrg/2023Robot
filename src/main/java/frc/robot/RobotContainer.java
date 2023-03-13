package frc.robot;


import frc.robot.commands.LimeLightVals;
import frc.robot.commands.outtakeTransferMovement;
import frc.robot.commands.pistonIntakeGrab;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.SwerveJoystickCommand;
import frc.robot.commands.armJoystickCommand;
import frc.robot.subsystems.LimeLight;
import frc.robot.subsystems.pistonIntake;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.linearSlideArm;
import frc.robot.subsystems.outtakeTransfer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;



public class RobotContainer {

  public SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
  public LimeLight limeLightSubsystem = new LimeLight();
  public linearSlideArm armSubsystem = new linearSlideArm();
  public pistonIntake pistonIntakeSubsystem = new pistonIntake();
  public outtakeTransfer outtakeTransferSubsystem = new outtakeTransfer();
  


  private final XboxController driverController = new XboxController(OIConstants.kDriverControllerPort);
  private final XboxController operatorController = new XboxController(OIConstants.kOperatorControllerPort);

  SendableChooser<Command> m_chooser = new SendableChooser<>();
  

  public RobotContainer() {

    swerveSubsystem.setDefaultCommand(new SwerveJoystickCommand(
      swerveSubsystem,
      // Left Joystick Field Oriented
      () -> -driverController.getRawAxis(OIConstants.kDriverYAxis),
      () -> driverController.getRawAxis(OIConstants.kDriverXAxis),

      //Right Joystick For Robot Centic
      () -> -driverController.getRawAxis(5),
      () -> driverController.getRawAxis(4),

      // Triggers for turning
      () -> driverController.getRawAxis(3),
      () -> driverController.getRawAxis(2),

      //Varied Assortment of Buttons to click
      () -> !driverController.getRawButton(OIConstants.kDriverFieldOrientedButtonIdx),


      () -> driverController.getRawButton(1),
      () -> driverController.getRawButton(2),
      () -> driverController.getRawButton(3),


      () -> driverController.getRawButton(OIConstants.kAlignWithTargetButton),
      () -> driverController.getRawButton(OIConstants.kResetDirectionButton)
    ));
     

    armSubsystem.setDefaultCommand(new armJoystickCommand(
      armSubsystem, 
      //Left Stick Y Axis
      () -> operatorController.getRawAxis(1)
    ));

    pistonIntakeSubsystem.setDefaultCommand(new pistonIntakeGrab(
      pistonIntakeSubsystem
    ));

    limeLightSubsystem.setDefaultCommand(new LimeLightVals(
      limeLightSubsystem
    ));

    outtakeTransferSubsystem.setDefaultCommand(new outtakeTransferMovement(
      outtakeTransferSubsystem
    ));

    configureButtonBindings();

  }

  private void configureButtonBindings() {
    //new JoystickButton(driverController, 2).whenPressed(() -> swerveSubsystem.zeroHeading());

    

  }


  public Command getAutonomousCommand() {

    //armSubsystem.midConeHeight();

    // Wheelie Should open

    //armSubsystem.resetHeight();

    // go to postion using odometry
    outtakeTransferSubsystem.outtakeMotor.set(.15);
    armSubsystem.highConeHeight();
    outtakeTransferSubsystem.outtakeMotor.set(-.15); 
    armSubsystem.resetHeight();
    outtakeTransferSubsystem.outtakeMotor.set(0);
    new Pose2d(0,0,Rotation2d.fromDegrees(180));
    swerveSubsystem.zeroHeading();
    new Translation2d(1.5,0);















    //return m_chooser.getSelected();
        //return m_chooser.getSelected();
    // 1. Create trajectory settings
    TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
      .65,
      .55)
        .setKinematics(DriveConstants.kDriveKinematics);

    // 2. Generate trajectory
    Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
      new Pose2d(0, 0, new Rotation2d(0)),
      List.of
        (
        new Translation2d(1.65, 0)
        ),
      new Pose2d(1.65, 0, Rotation2d.fromDegrees(0)),
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