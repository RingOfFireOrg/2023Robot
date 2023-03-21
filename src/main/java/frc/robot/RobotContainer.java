package frc.robot;


import frc.robot.Auto.ArmExtend;
import frc.robot.Auto.AutoCommandBuffer;
import frc.robot.Auto.wheelieGripSet;
import frc.robot.Auto.whilePitchCMD;
import frc.robot.commands.AutoFactory;
import frc.robot.commands.LimeLightVals;
import frc.robot.commands.SwerveJoystickCommand;
import frc.robot.commands.armJoystickCommand;
import frc.robot.commands.outtakeTransferMovement;
import frc.robot.commands.pistonIntakeGrab;
import java.util.List;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Auto.PistonEncoderMovement;
import frc.robot.Auto.pistonOpenClose;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.DriveConstants.OIConstants;
import frc.robot.subsystems.LimeLight;
import frc.robot.subsystems.pistonIntake;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.linearSlideArm;
import frc.robot.subsystems.outtakeTransfer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;




public class RobotContainer {

  public SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
  public LimeLight limeLightSubsystem = new LimeLight();
  public linearSlideArm armSubsystem = new linearSlideArm();
  public pistonIntake pistonIntakeSubsystem = new pistonIntake();
  public outtakeTransfer outtakeTransferSubsystem = new outtakeTransfer();
  public AutoFactory autoFactory;
  


  private final XboxController driverController = new XboxController(OIConstants.kDriverControllerPort);
  private final XboxController operatorController = new XboxController(OIConstants.kOperatorControllerPort);

  SendableChooser<Command> m_chooser = new SendableChooser<>();

  public RobotContainer() {
    


    // m_chooser.setDefaultOption("Move Straight", auto1());
    // m_chooser.addOption("Charge Station", auto2());
    SmartDashboard.putData(m_chooser);


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
      () -> driverController.getRawButton(4),


      () -> driverController.getRawButton(OIConstants.kAlignWithTargetButton),
      () -> driverController.getRawButton(OIConstants.kResetDirectionButton)
    ));
     

    armSubsystem.setDefaultCommand(new armJoystickCommand(
      armSubsystem, 
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













  private final Command auto1(){
    //1. Create trajectory settings
    TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
      .65,
      .55)
        .setKinematics(DriveConstants.kDriveKinematics);

    // 2. Generate trajectory
    Trajectory trajectory = TrajectoryGenerator.generateTrajectory
    (
      new Pose2d(0, 0, new Rotation2d(0)),
      List.of (new Translation2d(1.63, 0)),

      new Pose2d(1.63, 0, Rotation2d.fromDegrees(0)),
      trajectoryConfig
    );




    // 3. Define PID controllers for tracking trajectory
    PIDController xController = new PIDController(frc.robot.Constants.DriveConstants.AutoConstants.kPXController, 0, 0);
    PIDController yController = new PIDController(frc.robot.Constants.DriveConstants.AutoConstants.kPYController, 0, 0);
    ProfiledPIDController thetaController = new ProfiledPIDController(
      frc.robot.Constants.DriveConstants.AutoConstants.kPThetaController, 0, 0, frc.robot.Constants.DriveConstants.AutoConstants.kThetaControllerConstraints);
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
      new WaitCommand(4),
      new InstantCommand(() -> swerveSubsystem.stopModules()));    
  }



  private final Command auto2() {


    TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
      .65,
      .55)
        .setKinematics(DriveConstants.kDriveKinematics);

    Trajectory trajectory1 = TrajectoryGenerator.generateTrajectory
    (
      new Pose2d(0, 0, new Rotation2d(0)),
      List.of (new Translation2d(1.4, 0)),

      new Pose2d(1.4, 0, Rotation2d.fromDegrees(0)),
      trajectoryConfig
    );


    PIDController xController = new PIDController(frc.robot.Constants.DriveConstants.AutoConstants.kPXController, 0, 0);
    PIDController yController = new PIDController(frc.robot.Constants.DriveConstants.AutoConstants.kPYController, 0, 0);
    ProfiledPIDController thetaController = new ProfiledPIDController(
      frc.robot.Constants.DriveConstants.AutoConstants.kPThetaController, 0, 0, frc.robot.Constants.DriveConstants.AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    // 4. Construct command to follow trajectory
    SwerveControllerCommand undershoot = new SwerveControllerCommand(
      trajectory1,
      swerveSubsystem::getPose,
      DriveConstants.kDriveKinematics,
      xController,
      yController,
      thetaController,
      swerveSubsystem::setModuleStates,
      swerveSubsystem);

      // Trajectory trajectory2 = TrajectoryGenerator.generateTrajectory
      // (
      //   new Pose2d(0, 0, new Rotation2d(0)),
      //   List.of (new Translation2d(0, 1.5)),
  
      //   new Pose2d(0, 1.5, Rotation2d.fromDegrees(0)),
      //   trajectoryConfig
      // );

      // SwerveControllerCommand questionableShoot = new SwerveControllerCommand(
      //   trajectory2,
      //   swerveSubsystem::getPose,
      //   DriveConstants.kDriveKinematics,
      //   xController,
      //   yController,
      //   thetaController,
      //   swerveSubsystem::setModuleStates,
      //   swerveSubsystem);
  

    // return new SequentialCommandGroup(
    //   new InstantCommand(() -> swerveSubsystem.resetOdometry(trajectory1.getInitialPose())),
    //   undershoot,
    //   new WaitCommand(2),
    //   new PIDAutoBalancer(swerveSubsystem),
    //   new InstantCommand(() -> swerveSubsystem.stopModules())
    //   );

    //use thiss to test if its actually working
    return new SequentialCommandGroup(
      new InstantCommand(() -> swerveSubsystem.resetOdometry(trajectory1.getInitialPose())),
      undershoot,
      // new InstantCommand(() -> swerveSubsystem.resetOdometry(trajectory2.getInitialPose())),
      // new WaitCommand(10),
      // questionableShoot,
      new InstantCommand(() -> swerveSubsystem.stopModules())
      );

  }


  private final Command auto3() {

    TrajectoryConfig trajectoryConfig = new TrajectoryConfig
    (
      .65,
      .55
      
    )
        .setKinematics(DriveConstants.kDriveKinematics);
    trajectoryConfig.setReversed(true);
        

    Trajectory reverseTrajectory1 = TrajectoryGenerator.generateTrajectory
    (
      new Pose2d(0, 0, new Rotation2d(0)),
      List.of (new Translation2d(1.4, 0)),
      new Pose2d(1.4, 0, Rotation2d.fromDegrees(0)),
      trajectoryConfig
    );


    PIDController xController = new PIDController(frc.robot.Constants.DriveConstants.AutoConstants.kPXController, 0, 0);
    PIDController yController = new PIDController(frc.robot.Constants.DriveConstants.AutoConstants.kPYController, 0, 0);
    ProfiledPIDController thetaController = new ProfiledPIDController(
      frc.robot.Constants.DriveConstants.AutoConstants.kPThetaController, 0, 0, frc.robot.Constants.DriveConstants.AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    // 4. Construct command to follow trajectory
    SwerveControllerCommand REVERSEundershoot = new SwerveControllerCommand(
      reverseTrajectory1,
      swerveSubsystem::getPose,
      DriveConstants.kDriveKinematics,
      xController,
      yController,
      thetaController,
      swerveSubsystem::setModuleStates,
      swerveSubsystem);

    






    return new SequentialCommandGroup(

      // Grip the Cube
      new wheelieGripSet(outtakeTransferSubsystem, -1),
      new pistonOpenClose(pistonIntakeSubsystem, "open"),
      new PistonEncoderMovement(pistonIntakeSubsystem, 0),

      // remove piston intake

      //Move Arm to high position
      new ArmExtend(armSubsystem, "high"),
      new AutoCommandBuffer(),

      //Open Wheelie
      new wheelieGripSet(outtakeTransferSubsystem, 1),
      new AutoCommandBuffer(),

      //After wheelie is loose, set it to 0 so it does not rotate forever ongong
      new wheelieGripSet(outtakeTransferSubsystem, 0),
      new AutoCommandBuffer(),

      //Move the arm back down
      new ArmExtend(armSubsystem, "reset"),
      new AutoCommandBuffer(),

      // Startup Trajectory and then PID loop balence on charge station
      new InstantCommand(() -> swerveSubsystem.resetOdometry(reverseTrajectory1.getInitialPose())),
      REVERSEundershoot,
      new InstantCommand(() -> swerveSubsystem.stopModules()));
  }






  
  
  public Command getAutonomousCommand() {

        return new InstantCommand(() ->
        swerveSubsystem.resetAndSetAngle(180)).andThen(autoFactory.getAutoRoutine());  

 }
}
