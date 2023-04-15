// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.linearSlideArm;
import frc.robot.subsystems.outtakeTransfer;
import frc.robot.subsystems.pistonIntake;
import frc.robot.Auto.AutoBuilder;
import frc.robot.Auto.FollowTrajectoryPathPlanner;
import frc.robot.Auto.SwerveAutoBuilder2;
import frc.robot.commands.CommandGroups.HighCubeDrop;
/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private RobotContainer m_robotContainer;
  private static SendableChooser<AutoModes> autoChooser;
  private AutoModes previousSelectedAuto;
  SwerveSubsystem swerveSubsystem;
  
  private AutoBuilder autoBuilder;

  linearSlideArm armSubsystem;
  pistonIntake pistonIntakeSubsystem ;
  outtakeTransfer outtakeTransferSubsystem;
  SwerveAutoBuilder2 builder;


  public enum AutoModes {
    AUTO1, AUTO2, AUTO3, AUTO4, AUTO5, AUTO6
  }
  
  @Override
  public void robotInit() {
    // HashMap<String, Command> eventMap = new HashMap<>();
    // eventMap.put("ScoreHigh", new HighCubeDrop(armSubsystem, outtakeTransferSubsystem, pistonIntakeSubsystem, swerveSubsystem));
    
    // SwerveAutoBuilder m_trajectory =  builder.newAutoBuilder(swerveSubsystem,eventMap);
    // ArrayList<PathPlannerTrajectory> pathGroup = (ArrayList<PathPlannerTrajectory>) PathPlanner.loadPathGroup("1meter",new PathConstraints(5, 5));

    // // Create and push Field2d to SmartDashboard.
    // Field2d m_field = new Field2d();
    // SmartDashboard.putData(m_field);

    // // Push the trajectory to Field2d.
    // m_field.getObject("traj").setTrajectory(m_trajectory.fullAuto(pathGroup));
    


    //     Thread m_visionThread = new Thread(
    //     () -> {
    //       UsbCamera camera = CameraServer.startAutomaticCapture();

    //       camera.setResolution(640, 480);
        

    //       CvSink cvSink = CameraServer.getVideo();

    //       CvSource outputStream = CameraServer.putVideo("Rectangle", 640, 480);

    //       Mat mat = new Mat();
          
    //       while (!Thread.interrupted()) {
    //         if (cvSink.grabFrame(mat) == 0) {

    //           outputStream.notifyError(cvSink.getError());
   
    //           continue;
    //         }
     
    //         Imgproc.rectangle(
    //             mat, new Point(100, 100), new Point(400, 400), new Scalar(255, 255, 255), 5);

    //         outputStream.putFrame(mat);
    //       }
    //     });
    // m_visionThread.setDaemon(true);
    // m_visionThread.start();
    m_robotContainer = new RobotContainer();

    autoChooser = new SendableChooser<>();
    autoChooser.setDefaultOption("AUTO1", AutoModes.AUTO1);
    autoChooser.addOption("AUTO2", AutoModes.AUTO2);
    autoChooser.addOption("AUTO3", AutoModes.AUTO3);
    autoChooser.addOption("AUTO4", AutoModes.AUTO4);
    autoChooser.addOption("AUTO5", AutoModes.AUTO5);
    autoChooser.addOption("AUTO6", AutoModes.AUTO6);

    SmartDashboard.putData("Auto Chooser", autoChooser);
    previousSelectedAuto = autoChooser.getSelected();


    
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();    
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }  
    // AutoBuilder autoBuilder = new AutoBuilder();
    // autoBuilder.setRobotContainer(m_robotContainer);
    // autoBuilder.setAutoMode(autoChooser.getSelected());
    // if (m_autonomousCommand != null) {
    //     m_autonomousCommand.schedule();
    // }

    // m_autonomousCommand = autoBuilder.build();

  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    //SmartDashboard.putNumber("Pose X", swerveSubsystem.getPose().getX());
    //SmartDashboard.putNumber("Pose", swerveSubsystem.);

  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {

  
  }

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}

  

}
