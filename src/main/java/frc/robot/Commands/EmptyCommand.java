package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;

public class EmptyCommand extends Command {

  @Override
  public void initialize() {}

  @Override
  public void execute() {}

  @Override
  public boolean isFinished() {
    return true;
  }

  @Override
  public void end(boolean interrupted) {
    System.out.println(interrupted);
  }
}
