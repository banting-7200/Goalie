package frc.robot.Vision;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Feedback.ShuffleboardSubsystem;

public class AprilTagCam extends SubsystemBase {
  ShuffleboardSubsystem shuffle = ShuffleboardSubsystem.getInstance();
  private NetworkTable table;
  private static AprilTagCam instance;

  private AprilTagCam() {
    table =
        NetworkTableInstance.getDefault()
            .getTable("photonvision")
            .getSubTable("Arducam_OV9281_USB_Camera"); // TODO: Make sure this is the right key
    shuffle.setTab("Camera Streams");
    shuffle.addCamera("AprilTagCamera", "PhotonVision", "http://photonvision.local:1183");
  }

  public static synchronized AprilTagCam getInstance() {
    if (instance == null) {
      instance = new AprilTagCam();
    }
    return instance;
  }

  public double getTagWidth() {
    return (double) table.getEntry("targetPixelsX").getNumber(-1);
  }

  public double getTagHeight() {
    return (double) table.getEntry("targetPixelsY").getNumber(-1);
  }

  public double getTagArea() {
    return (double) table.getEntry("targetArea").getNumber(-1);
  }

  public double getTagYaw() {
    return (double) table.getEntry("targetYaw").getNumber(0);
  }

  public double getTagPitch() {
    return (double) table.getEntry("targetPitch").getNumber(-10);
  }

  public boolean hasTarget() {
    return table.getEntry("hasTarget").getBoolean(false);
  }

  public void updateShuffle() {
    shuffle.setTab("Camera Streams");

    shuffle.setLayout("AprilTagCam");
    shuffle.setNumber("width", getTagWidth());
    shuffle.setNumber("height", getTagHeight());
    shuffle.setNumber("area", getTagArea());
    shuffle.setBoolean("target", hasTarget());
    shuffle.setNumber("yaw", getTagYaw());
  }
}
