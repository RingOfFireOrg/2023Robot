// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.Joystick;
//import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.SwerveJoystickCommand;
import frc.robot.subsystems.SwerveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
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

  private static final int MathPI = 0;

  public SwerveSubsystem swerveSubsystem = new SwerveSubsystem();

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
        () -> driverController.getRawButton(OIConstants.kResetDirectionButton),

        () -> operatorController.getRawButton(OIConstants.kRotate0Button),
        () -> operatorController.getRawButton(OIConstants.kRotate180Button),
        () -> operatorController.getRawButton(OIConstants.kExtendFullButton),
        () -> operatorController.getRawButton(OIConstants.kRetractButton),
        () -> operatorController.getRawButton(OIConstants.kToggleGrabButton),
        () -> operatorController.getRawButton(OIConstants.kReverseGrabButton),
        () -> operatorController.getRawButton(OIConstants.kForwardGrabButton),
        () -> operatorController.getRawButton(OIConstants.kManuelButton)
        ));

    configureButtonBindings();
  }

  private void configureButtonBindings() {
    //new JoystickButton(driverController, 2).whenPressed(() -> swerveSubsystem.zeroHeading());
  }
  
}
