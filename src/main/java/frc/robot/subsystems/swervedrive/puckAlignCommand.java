package frc.robot.subsystems.swervedrive;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Vision.LimelightDevice;

public class puckAlignCommand extends Command {

  private SwerveSubsystem swerve;
  private LimelightDevice limelight;

  private double puckX;
  private double puckY;

  private double alignSpeed;

  private boolean puckDetected;

  public puckAlignCommand(SwerveSubsystem swerve, LimelightDevice limelight) {
    this.swerve = swerve;
    this.limelight = limelight;
    addRequirements(swerve, limelight);
  }

  @Override
  public void execute() {
    getTagData();
    if (puckDetected) {
      swerve.drive(new Translation2d(getXtranslation(), getYtranslation()), getRotation(), false);
    } else {
      swerve.drive(new Translation2d(0, 0), getRotation(), false);
    }
  }

  private double getXtranslation() {
    if (puckY > 2) return alignSpeed;
    if (puckY < -2) return -alignSpeed;
    return 0;
  }

  private double getYtranslation() {
    if (puckX > 2) return alignSpeed;
    if (puckX < -2) return -alignSpeed;
    return 0;
  }

  private double getRotation() {
    if (swerve.getHeading().getDegrees() > 1) return -alignSpeed;
    if (swerve.getHeading().getDegrees() < -1) return alignSpeed;
    return 0;
  }

  private void getTagData() {
    puckY = limelight.getTagY();
    puckX = limelight.getTagX();
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {}
}
