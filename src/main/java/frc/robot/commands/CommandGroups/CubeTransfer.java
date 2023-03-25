// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.CommandGroups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AutoCommands.PistonIntakeMovement;
import frc.robot.commands.AutoCommands.PistonIntakeStatus;
import frc.robot.subsystems.pistonIntake;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class CubeTransfer extends SequentialCommandGroup {
  pistonIntake piston;
  public CubeTransfer() {
    addRequirements(piston);
    addCommands
    (
      // wheelie up encoder
      new PistonIntakeStatus(piston, "close"),
      new PistonIntakeMovement(piston, "up")
      //wheeli down

    );
  }
}
