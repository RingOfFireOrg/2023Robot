// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.CommandGroups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AutoCommands.PistonIntakeMovement;
import frc.robot.commands.AutoCommands.PistonIntakeStatus;
import frc.robot.subsystems.pistonIntake;

public class LowCubeDrop extends SequentialCommandGroup {
  pistonIntake piston;
  public LowCubeDrop() {
    addRequirements(piston);
    addCommands
    (

      new PistonIntakeMovement(piston, "down"),
      new PistonIntakeStatus(piston, "open"),
      new PistonIntakeMovement(piston, "up")

    );
  }
}
