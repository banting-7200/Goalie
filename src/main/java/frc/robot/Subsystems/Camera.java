package frc.robot.Subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Camera extends SubsystemBase {
  ShuffleboardSubsystem shuffle = ShuffleboardSubsystem.getInstance();
  private NetworkTable table;
  private static Camera instance;

  private Camera() {
    table =
        NetworkTableInstance.getDefault()
            .getTable("photonvision")
            .getSubTable("Arducam_OV9281_USB_Camera"); // TODO: Make sure this is the right key
    shuffle.setTab("Camera Streams");
    shuffle.addCamera("Camera", "PhotonVision", "http://photonvision.local:1183");
  }

  public static synchronized Camera getInstance() {
    if (instance == null) {
      instance = new Camera();
    }
    return instance;
  }

  public double getTargetWidth() {
    return (double) table.getEntry("targetPixelsX").getNumber(-1);
  }

  public double getTargetHeight() {
    return (double) table.getEntry("targetPixelsY").getNumber(-1);
  }

  public double getTargetArea() {
    return (double) table.getEntry("targetArea").getNumber(-1);
  }

  public double getTargetYaw() {
    return (double) table.getEntry("targetYaw").getNumber(0);
  }

  public double getTargetPitch() {
    return (double) table.getEntry("targetPitch").getNumber(-10);
  }

  public boolean hasTarget() {
    return table.getEntry("hasTarget").getBoolean(false);
  }

  public void updateShuffle() {
    shuffle.setTab("Camera Streams");

    shuffle.setLayout("Camera");
    shuffle.setNumber("width", getTargetWidth());
    shuffle.setNumber("height", getTargetHeight());
    shuffle.setNumber("area", getTargetArea());
    shuffle.setBoolean("target", hasTarget());
    shuffle.setNumber("yaw", getTargetYaw());
  }
}
