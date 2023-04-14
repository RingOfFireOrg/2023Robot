package frc.robot;


import frc.robot.commands.LimeLightVals;
import frc.robot.commands.AutoCommands.ArmAutoMovement;
import frc.robot.commands.AutoCommands.PistonIntakeMovement;
import frc.robot.commands.AutoCommands.PistonIntakeStatus;
import frc.robot.commands.AutoCommands.TransferGrip;
import frc.robot.commands.CommandGroups.HighCubeDrop;
import frc.robot.commands.CommandGroups.MidCubeDrop;
import frc.robot.commands.TeleopCommands.SwerveJoystickCommand;
import frc.robot.commands.TeleopCommands.armJoystickCommand;
import frc.robot.commands.TeleopCommands.outtakeTransferMovement;
import frc.robot.commands.TeleopCommands.pistonIntakeGrab;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import frc.robot.commands.CommandGroups.HighCubeDrop;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Auto.AutoBuilder;
import frc.robot.Auto.FollowTrajectoryPathPlanner;
import frc.robot.Auto.PIDAutoBalancer;
import frc.robot.Auto.PPSwerveAutoBuild;
import frc.robot.Auto.ReversePIDAutoBalancer;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.LimeLight;
import frc.robot.subsystems.pistonIntake;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.linearSlideArm;
import frc.robot.subsystems.outtakeTransfer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;


public class RobotContainer {

  public SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
  public LimeLight limeLightSubsystem = new LimeLight();
  public linearSlideArm armSubsystem = new linearSlideArm();
  public pistonIntake pistonIntakeSubsystem = new pistonIntake();
  public outtakeTransfer outtakeTransferSubsystem = new outtakeTransfer();

  public HashMap<String, Command> eventMap = new HashMap<>();


//                                 ADD A REVERSE BAALENCE TO EVENT MAP


  private final XboxController driverController = new XboxController(OIConstants.kDriverControllerPort);
  private final XboxController operatorController = new XboxController(OIConstants.kOperatorControllerPort);

  SendableChooser<Command> m_chooser = new SendableChooser<>();

  public RobotContainer() {
    
    // eventMap.put("ScoreHigh", new HighCubeDrop(armSubsystem, outtakeTransferSubsystem, pistonIntakeSubsystem, swerveSubsystem));
    // eventMap.put("NormalBalence", new PIDAutoBalancer(swerveSubsystem));

    // // Create the AutoBuilder. This only needs to be created once when robot code starts, not every time you want to create an auto command. A good place to put this is in RobotContainer along with your subsystems.
    // SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder
    // (
    //   swerveSubsystem::getPose, // Pose2d supplier
    //   swerveSubsystem::resetOdometry, // Pose2d consumer, used to reset odometry at the beginning of auto
    //   DriveConstants.kDriveKinematics,
    //   new PIDConstants(0.4, 0.0, 0), // PID constants to correct for translation error (used to create the X and Y PID controllers)
    //   new PIDConstants(1.75, 0.0, 0), // PID constants to correct for rotation error (used to create the rotation controller)
    //   swerveSubsystem::setModuleStates, // Module states consumer used to output to the drive subsystem
    //   eventMap,
    //   true, 
    //   swerveSubsystem 
    // );



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












  private final Command auto2() {


    TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
      .65,
      .55)
        .setKinematics(DriveConstants.kDriveKinematics);

    Trajectory trajectory1 = TrajectoryGenerator.generateTrajectory
    (
      new Pose2d(0, 0, new Rotation2d(0)),
      List.of (new Translation2d(1, 0)),

      new Pose2d(1, 0, Rotation2d.fromDegrees(0)),
      trajectoryConfig
    );


    PIDController xController = new PIDController(AutoConstants.kPXController, 0, 0);
    PIDController yController = new PIDController(AutoConstants.kPYController, 0, 0);
    ProfiledPIDController thetaController = new ProfiledPIDController(
      AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
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


    //use thiss to test if its actually working
    return new SequentialCommandGroup(
      new InstantCommand(() -> swerveSubsystem.resetOdometry(trajectory1.getInitialPose())),
      undershoot,
      new PIDAutoBalancer(swerveSubsystem),
      // new InstantCommand(() -> swerveSubsystem.resetOdometry(trajectory2.getInitialPose())),
      // new WaitCommand(10),
      // questionableShoot,
      new InstantCommand(() -> swerveSubsystem.stopModules())
      );



  }




  


  public Command midAutoHigh() {
    return new SequentialCommandGroup
    (
      new HighCubeDrop(armSubsystem, outtakeTransferSubsystem, pistonIntakeSubsystem, swerveSubsystem),

      new PIDAutoBalancer(swerveSubsystem)

    );
  }

  public Command trajectoryTestNew(SwerveAutoBuilder autoBuilder) {

    ArrayList<PathPlannerTrajectory> pathGroup = (ArrayList<PathPlannerTrajectory>) PathPlanner.loadPathGroup("1meterback",new PathConstraints(1, 1));
    autoBuilder.fullAuto(pathGroup).schedule();
    return new InstantCommand();
  
  }


  
  public Command getAutonomousCommand() {


    //return new HighCubeDrop(armSubsystem, outtakeTransferSubsystem, pistonIntakeSubsystem, swerveSubsystem);


    //return new FollowTrajectoryPathPlanner(swerveSubsystem, "PIDTesting4", true);

    return new SequentialCommandGroup 
    (
      //new HighCubeDrop(armSubsystem, outtakeTransferSubsystem, pistonIntakeSubsystem, swerveSubsystem),
      //new FollowTrajectoryPathPlanner(swerveSubsystem, "3.99Meters", true,1,1)
      //new FollowTrajectoryPathPlanner(swerveSubsystem, "PIDTesting4", true,1,1,false),
      //new FollowTrajectoryPathPlanner(swerveSubsystem, "PIDTesting5", false,1,1,false),
      //new FollowTrajectoryPathPlanner(swerveSubsystem, "PIDTesting6", false,1,1,false),
      //new PPSwerveAutoBuild(swerveSubsystem, armSubsystem, outtakeTransferSubsystem, pistonIntakeSubsystem,"2meterback",1,1)

      new ReversePIDAutoBalancer(swerveSubsystem)
    );

    //return auto2();


    //return m_chooser.getSelected();
    // return new SequentialCommandGroup(
    //   new PistonIntakeStatus(pistonIntakeSubsystem, "open"),
    //   new PistonIntakeMovement(pistonIntakeSubsystem, "down"),
    //   new TransferGrip(outtakeTransferSubsystem, "close"),
    //   new ArmAutoMovement(armSubsystem, "high"),
    //   new TransferGrip(outtakeTransferSubsystem, "open"),
    //   new ArmAutoMovement(armSubsystem, "reset"),
    //   new InstantCommand(() -> swerveSubsystem.stopModules())
    //   );
    //return m_chooser.getSelected();
    //return auto2();
    //return new HighCubeDrop(armSubsystem, outtakeTransferSubsystem, pistonIntakeSubsystem, swerveSubsystem);
  }
}
