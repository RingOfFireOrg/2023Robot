package frc.robot.commands.CommandGroups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AutoCommands.ArmAutoMovement;
import frc.robot.commands.AutoCommands.TransferGrip;
import frc.robot.subsystems.linearSlideArm;
import frc.robot.subsystems.outtakeTransfer;

public class MidCubeDrop extends SequentialCommandGroup {
  linearSlideArm arm;
  outtakeTransfer wheelie;
  public MidCubeDrop() {
    addRequirements(wheelie);
    addRequirements(arm);
    
    addCommands
    (
      new TransferGrip(wheelie, "close"),
      new ArmAutoMovement(arm, "mid"),
      new TransferGrip(wheelie, "open"),
      new ArmAutoMovement(arm, "reset")      
    );
  }
}
