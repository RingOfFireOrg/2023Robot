// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Auto;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;

/** Add your docs here. */
public class trajectoryTest {
    public static Trajectory trajectory1(TrajectoryConfig trajectoryConfig){
        Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
            new Pose2d(0, 0, new Rotation2d(0)), 
            List.of(
              new Translation2d(1,0),
              new Translation2d(1,-1)
              
              ),
          new Pose2d(2, -1, Rotation2d.fromDegrees(180)),
          trajectoryConfig);
        return trajectory;

  }
}
