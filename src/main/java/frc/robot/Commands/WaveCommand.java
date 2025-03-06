package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.ArmSubsystem;
import frc.robot.Subsystems.LightsSubsystem;

public class WaveCommand extends Command {

  double position = 0;
  double increase = 0.05;
  double time = 0;
  double waveTime = 3.5;

  ArmSubsystem arm;
  LightsSubsystem lights;

  public WaveCommand(ArmSubsystem arm, LightsSubsystem lights) {
    this.arm = arm;
    this.lights = lights;
  }

  @Override
  public void initialize() {
    System.out.println("Starting Wave Command");
  }

  @Override
  public void execute() {
    lights.rainbow();
    position += increase;
    if (Math.abs(position) >= 1) {
      increase *= -1;
    }
    time += 0.02;
    arm.moveFromRangeWave(-1, 1, position);
  }

  @Override
  public boolean isFinished() {
    return time > waveTime;
  }

  @Override
  public void end(boolean interrupted) {
    arm.moveToDownPosition();
    lights.solidColor(0, 0, 255);
  }
}
