package frc.robot.commands.CommandGroups;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AutoCommands.ArmAutoMovement;
import frc.robot.commands.AutoCommands.PistonIntakeMovement;
import frc.robot.commands.AutoCommands.PistonIntakeStatus;
import frc.robot.commands.AutoCommands.TransferGrip;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.linearSlideArm;
import frc.robot.subsystems.outtakeTransfer;
import frc.robot.subsystems.pistonIntake;

public class HighCubeDrop extends SequentialCommandGroup {

  public HighCubeDrop
  (
    linearSlideArm arm,
    outtakeTransfer outtakeTransferSubsystem,
    pistonIntake pistonIntakeSubsystem,
    SwerveSubsystem swerveSubsystem
  ) 
  {

    addCommands
    (
      new PistonIntakeStatus(pistonIntakeSubsystem, "open"),
      new PistonIntakeMovement(pistonIntakeSubsystem, "down"),
      new TransferGrip(outtakeTransferSubsystem, "close"),
      new ArmAutoMovement(arm, "high"),
      new TransferGrip(outtakeTransferSubsystem, "open"),
      new ArmAutoMovement(arm, "reset"),
      new InstantCommand(() -> swerveSubsystem.stopModules())
    );
  }
}
