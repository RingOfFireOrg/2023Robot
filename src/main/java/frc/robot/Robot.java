// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import java.util.List;

import javax.inject.Inject;

import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import com.fasterxml.jackson.annotation.JacksonInject;
import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.SwerveSubsystem;
import edu.wpi.first.wpilibj.SerialPort;

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
  private Command autonomousCommand;
  @Inject
  SwerveAutoBuilder autoBuilder;

  public enum AutoModes 
  {
    LeaveCommunity
    (
      "",
      new PathConstraints(0.9, 0.5)
    ), 
    DistanceBasedChargeStation    
    (
      "",
      new PathConstraints(0.9, 0.7)
    ), 
    PIDAutoBalance    
    (
      "",
      new PathConstraints(0.75, 0.4)
    );
    
    public final String name;
    public final PathConstraints initConstraint;
    public final PathConstraints[] pathConstraints;

    AutoModes(String name, PathConstraints initConstraint, PathConstraints... pathConstraints) {
        this.name = name;
        this.initConstraint = initConstraint;
        this.pathConstraints = pathConstraints;
    }
  }
  
  @Override
  public void robotInit() {

    m_robotContainer = new RobotContainer();

    autoChooser = new SendableChooser<>();
    autoChooser.setDefaultOption("Leave Community", AutoModes.LeaveCommunity);
    autoChooser.addOption("Distance Based Charge Station", AutoModes.DistanceBasedChargeStation);
    autoChooser.addOption("PID Auto Balance", AutoModes.PIDAutoBalance);


    SmartDashboard.putData("Auto Chooser", autoChooser);
    previousSelectedAuto = autoChooser.getSelected();

    List<PathPlannerTrajectory> trajectory = PathPlanner.loadPathGroup(
      "ChargeStation", previousSelectedAuto.initConstraint, previousSelectedAuto.pathConstraints);
    autonomousCommand = autoBuilder.fullAuto(trajectory);

    
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
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

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
