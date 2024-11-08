package frc.robot.subsystems.swervedrive;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Vision.AprilTagCam;
import frc.robot.Vision.LimelightDevice;

public class alignCommand extends Command {
  AprilTagCam camera;
  SwerveSubsystem swerve;
  boolean useLimelight;
  LimelightDevice limelight = LimelightDevice.getInstance();
  private double tagX;
  private double tagY;
  private double tagArea;
  private double tagID;

  public alignCommand(SwerveSubsystem swerve, Boolean useLimelight) {
    camera = AprilTagCam.getInstance();
    this.swerve = swerve;
    this.useLimelight = useLimelight;
    addRequirements(swerve, camera);
  }

  @Override
  public void execute() {
    if (useLimelight) {
      readAprilTag();
      if (limelight.tagDetected()) {
        swerve.drive(
            new Translation2d(moveToAprilTagOffsetY(), moveToAprilTagOffsetX()),
            getRotation(),
            false);
      } else {
        swerve.drive(new Translation2d(0, 0), getRotation(), false);
      }

    } else {
      if (camera.hasTarget()) {
        swerve.drive(new Translation2d(getYtranslation(), getXtranslation()), getRotation(), false);
      } else {
        swerve.drive(new Translation2d(0, 0), getRotation(), false);
      }
    }
  }

  private double getXtranslation() {
    if (camera.getTagYaw() < -2) {
      return -0.1;
    } else if (camera.getTagYaw() > 2) {
      return 0.1;
    }
    return 0;
  }

  private double getYtranslation() {
    if (camera.getTagPitch() < -11) {
      return -0.1;
    } else if (camera.getTagPitch() > -9) {

      return 0.1;
    }
    return 0;
  }

  private double getRotation() {
    if (swerve.getHeading().getDegrees() > 1) {
      return -0.1;
    } else if (swerve.getHeading().getDegrees() < -1) {
      return 0.1;
    }
    return 0;
  }

  private double moveToAprilTagOffsetX() {
    if (tagX > 2) {
      return 0.2;
    } else if (tagX < -2) {
      return -0.2;
    }
    return 0;
  }

  private double moveToAprilTagOffsetY() {
    if (tagY > 2) {
      return 0.2;
    } else if (tagY < -2) {
      return -0.1;
    }
    return 0;
  }

  private void readAprilTag() {
    // Read all basic data from the limelight and assign them to local variables
    this.tagX = limelight.getTagX();
    this.tagY = limelight.getTagY();
    this.tagArea = limelight.getTagArea();
    this.tagID = limelight.getTagID();
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {}
}
