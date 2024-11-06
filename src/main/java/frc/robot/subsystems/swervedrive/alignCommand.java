package frc.robot.subsystems.swervedrive;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Vision.AprilTagCam;

public class alignCommand extends Command {
  AprilTagCam camera;
  SwerveSubsystem swerve;
  boolean run;

  public alignCommand(SwerveSubsystem swerve) {
    camera = AprilTagCam.getInstance();
    this.swerve = swerve;
    addRequirements(swerve, camera);
  }

  @Override
  public void execute() {
    if (camera.hasTarget()) {
      swerve.drive(new Translation2d(getYtranslation(), getXtranslation()), getRotation(), false);
    } else {
      swerve.drive(new Translation2d(0, 0), getRotation(), false);
    }
  }

  private double getXtranslation() {
    if (camera.getTagYaw() < -1) {
      return -0.1;
    } else if (camera.getTagYaw() > 1) {
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

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {}
}
