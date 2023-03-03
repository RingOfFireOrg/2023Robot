package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.linearSlideArm;

public class armJoystickCommand extends CommandBase {
  /** Creates a new armJoystickCommand. */

  linearSlideArm arm;
  double gamepadRightYValue;
  boolean aButton;
  boolean bButton;

  public armJoystickCommand(linearSlideArm arm, double gamepadRightYValue, boolean aButton, boolean bButton) {
    addRequirements(arm);
    this.arm = arm;
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    arm.armMovement(gamepadRightYValue,aButton,bButton);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
