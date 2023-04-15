// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Auto;

import java.util.HashMap;

import com.pathplanner.lib.auto.PIDConstants;

import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.SwerveSubsystem;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

/** Add your docs here. */
public class SwerveAutoBuilder2 {

    public SwerveAutoBuilder newAutoBuilder(SwerveSubsystem driveSubsystem, HashMap<String, Command> eventMap) {
        SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder
        (
            driveSubsystem::getPose, // Pose2d supplier
            driveSubsystem::resetOdometry3,
            DriveConstants.kDriveKinematics,
            new PIDConstants(5, 0.0, 0), // PID constants to correct for translation error (used to create the X and Y PID controllers)
            new PIDConstants(1.75, 0.0, 0), // PID constants to correct for rotation error (used to create the rotation controller)
            driveSubsystem::setModuleStates,
            eventMap,
            true,
            driveSubsystem
        );
        return autoBuilder;
    }


}
