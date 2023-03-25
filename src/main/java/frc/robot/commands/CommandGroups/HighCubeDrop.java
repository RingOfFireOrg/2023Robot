package frc.robot.commands.CommandGroups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AutoCommands.ArmAutoMovement;
import frc.robot.commands.AutoCommands.TransferGrip;
import frc.robot.subsystems.linearSlideArm;
import frc.robot.subsystems.outtakeTransfer;

public class HighCubeDrop extends SequentialCommandGroup {
  linearSlideArm arm;
  outtakeTransfer wheelie;
  public HighCubeDrop() {
    addRequirements(wheelie);
    addRequirements(arm);
    addCommands
    (
      new TransferGrip(wheelie, "close"),
      new ArmAutoMovement(arm, "high"),
      new TransferGrip(wheelie, "open"),
      new ArmAutoMovement(arm, "reset")
    );
  }
}
