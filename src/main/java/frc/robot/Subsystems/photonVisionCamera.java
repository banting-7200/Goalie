package frc.robot.Subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class photonVisionCamera extends SubsystemBase {
  private NetworkTable table;

  public photonVisionCamera(String cameraName) {
    table = NetworkTableInstance.getDefault().getTable("photonvision").getSubTable(cameraName);
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

  public void test() {
    System.out.println("Yaw:" + getTargetYaw() + "Pitch" + getTargetPitch());
  }
}
