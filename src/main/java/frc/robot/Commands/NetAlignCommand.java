package frc.robot.Commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.SwerveSubsystem;
import frc.robot.Vision.PhotonVisionCamera;

public class NetAlignCommand extends Command {

  private SwerveSubsystem swerve;
  private PhotonVisionCamera camera;

  private double tagYaw;
  private double tagPitch;

  private double speed;

  private boolean tagDetected;

  public NetAlignCommand(SwerveSubsystem swerve, PhotonVisionCamera camera) {
    this.camera = camera;
    this.swerve = swerve;
  }

  @Override
  public void execute() {
    getTagData();
    if (tagDetected) {
      if (getXtranslation() == 0 && getYtranslation() == 0 && getRotation() == 0) end(false);
      swerve.drive(new Translation2d(getXtranslation(), getYtranslation()), getRotation(), false);
    } else {
      end(false);
    }
  }

  private double getXtranslation() {
    if (tagPitch < -11) return -speed;
    if (tagPitch > -9) return speed;
    return 0;
  }

  private double getYtranslation() {
    if (tagYaw < -2) return -speed;
    if (tagYaw > 2) return speed;
    return 0;
  }

  private double getRotation() {
    if (swerve.getHeading().getDegrees() > 1) return -speed;
    if (swerve.getHeading().getDegrees() < -1) return speed;
    return 0;
  }

  private void getTagData() {
    tagYaw = camera.getTargetYaw();
    tagPitch = camera.getTargetPitch();
    tagDetected = camera.hasTarget();
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    swerve.drive(new Translation2d(0, 0), getRotation(), false);
  }
}
