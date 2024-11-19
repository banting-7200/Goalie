package frc.robot.subsystems.swervedrive;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Vision.AprilTagCam;
import frc.robot.Vision.LimelightDevice;

public class netAlignCommand extends Command {

  private SwerveSubsystem swerve;
  private AprilTagCam camera;
  private LimelightDevice limelight;

  private boolean useLimelight;

  private double tagX;
  private double tagY;

  private double tagYaw;
  private double tagPitch;

  private double speed;

  private boolean tagDetected;

  public netAlignCommand(SwerveSubsystem swerve, AprilTagCam camera) {
    this.camera = camera;
    this.swerve = swerve;
    useLimelight = false;
    addRequirements(swerve, camera);
  }

  public netAlignCommand(SwerveSubsystem swerve, LimelightDevice limelight) {
    this.swerve = swerve;
    this.limelight = limelight;
    useLimelight = true;
    addRequirements(swerve, camera);
  }

  @Override
  public void execute() {
    getTagData();
    if (tagDetected) {
      swerve.drive(new Translation2d(getXtranslation(), getYtranslation()), getRotation(), false);
    } else {
      swerve.drive(new Translation2d(0, 0), getRotation(), false);
    }
  }

  private double getXtranslation() {
    if (useLimelight) {
      if (tagY > 2) return speed;
      if (tagY < -2) return -speed;
    } else {
      if (tagPitch < -11) return -speed;
      if (tagPitch > -9) return speed;
    }
    return 0;
  }

  private double getYtranslation() {
    if (useLimelight) {
      if (tagX > 2) return speed;
      if (tagX < -2) return -speed;
    } else {
      if (tagYaw < -2) return -speed;
      if (tagYaw > 2) return speed;
    }
    return 0;
  }

  private double getRotation() {
    if (swerve.getHeading().getDegrees() > 1) return -speed;
    if (swerve.getHeading().getDegrees() < -1) return speed;
    return 0;
  }

  private void getTagData() {
    // Read all basic data from the limelight and assign them to local variables
    if (useLimelight) {
      tagX = limelight.getTagX();
      tagY = limelight.getTagY();
      tagDetected = limelight.tagDetected();
    } else {
      tagYaw = camera.getTagYaw();
      tagPitch = camera.getTagPitch();
      tagDetected = camera.hasTarget();
    }
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {}
}
