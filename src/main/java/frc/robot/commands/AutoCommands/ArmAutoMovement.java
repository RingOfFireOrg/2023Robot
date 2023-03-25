package frc.robot.commands.AutoCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.linearSlideArm;

public class ArmAutoMovement extends CommandBase {
  linearSlideArm arm;
  String movementLevel;
  Boolean armFinshCheck;

  public ArmAutoMovement(linearSlideArm arm, String movementLevel) {
    addRequirements(arm);
    this.arm = arm;
    this.movementLevel = movementLevel;
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {

    if (movementLevel == "reset") {
      armFinshCheck = arm.resetHeightReturn();
    }
    else if (movementLevel == "mid") {
      armFinshCheck = arm.midCubeHeightReturn();
    }
    else if (movementLevel == "high") {
      armFinshCheck = arm.highCubeHeightReturn();
    }

  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    if(armFinshCheck == true) {
      return true;
    }
    else {
      return false;
    }
  }
}
