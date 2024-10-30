package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LimelightDevice extends SubsystemBase {
  private NetworkTable mainTable;
  public int mode;
  private static LimelightDevice instance = null;
  public int speakerMiddleTag;
  public int speakerSideTag;
  public int ampTag;

  public LimelightDevice() {
    mainTable =
        NetworkTableInstance.getDefault().getTable("limelight"); // gets the network table with key

    mode = 0;
  }

  public static synchronized LimelightDevice getInstance() {
    if (instance == null) {
      instance = new LimelightDevice();
    }
    return instance;
  }

  public void setLight(boolean on) { // toggles lights (true for on, false for off)
    mainTable.getEntry("ledMode").setNumber(on ? 3 : 1);
  }

  public boolean getLight() {
    double light;
    light = (double) mainTable.getEntry("lightMode").getNumber(3);
    return light == 1 ? false : true;
  }

  public void setMode(int selection) { // sets pipeline
    mainTable.getEntry("pipeline").setNumber(selection);
  }

  public boolean tagDetected() { // returns true if tag is detected
    double ttarget = 0;
    try {
      ttarget = mainTable.getEntry("tv").getDouble(0);
    } catch (NullPointerException e) {
      System.out.println("ttarget ERROR, EXCEPTION: " + e);
      ttarget = 0;
    }
    boolean tdetected = ttarget == 0 ? false : true;
    return tdetected;
  }

  public int getTagID() { // returns id of apriltag or -1 if no tag is detected.
    int tid = 0;
    try {
      tid = (int) mainTable.getEntry("tid").getDouble(-1);
    } catch (NullPointerException e) {
      System.out.println("tid ERROR, EXCEPTION: " + e);
      tid = -1;
    }
    return tid;
  }

  public double getTagArea() { // return tag area
    double ta = 0;
    try {
      ta = mainTable.getEntry("ta").getDouble(0);

    } catch (NullPointerException e) {
      System.out.println("Tag Area ERROR, EXCEPTION: " + e);
      ta = 0;
    }
    return ta;
  }

  public double getTagX() { // return tag x value (horizontal across camera screen)
    double tx = 0;
    try {
      tx = mainTable.getEntry("tx").getDouble(0);
    } catch (NullPointerException e) {
      System.out.println("tx ERROR, EXCEPTION: " + e);
      tx = 0;
    }
    return tx;
  }

  public double getTagY() { // return tag y value (vertical across camera screen)
    double ty = 0;
    try {
      ty = mainTable.getEntry("ty").getDouble(0);
    } catch (NullPointerException e) {
      System.out.println("ty ERROR, EXCEPTION: " + e);
      ty = 0;
    }

    return ty;
  }
}
