package frc.robot.commands.TeleopCommands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.linearSlideArm;

public class armJoystickCommand extends CommandBase {
  /** Creates a new armJoystickCommand. */

  linearSlideArm arm;
  Supplier<Double> gamepadRightYValue;
  Supplier<Boolean> automateHighButton;

  public armJoystickCommand(linearSlideArm arm, Supplier<Double> gamepadRightYValue) {
    addRequirements(arm);
    this.arm = arm;
    
  }

  @Override
  public void initialize() {
    arm.encoderReset();

  }

  @Override
  public void execute() {
    arm.commandOrder();
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
