package frc.robot.Vision;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class LimelightCommands extends Command {
  // Creates a instance of the LimelightDevice class, instance ensures only 1 instance of
  // limelightdevice is created
  SwerveSubsystem swerve;
  LimelightDevice limelight = LimelightDevice.getInstance();
  private double tagX;
  private double tagY;
  private double tagArea;
  private double tagID;

  public LimelightCommands(SwerveSubsystem swerve) {
    this.swerve = swerve;
    addRequirements(swerve, limelight);
  }

  private void readAprilTag() {
    // Read all basic data from the limelight and assign them to local variables
    this.tagX = limelight.getTagX();
    this.tagY = limelight.getTagY();
    this.tagArea = limelight.getTagArea();
    this.tagID = limelight.getTagID();
  }

  @Override
  public void execute() {}
}
