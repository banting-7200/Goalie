package frc.robot.Vision;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Feedback.ShuffleboardSubsystem;

public class LimelightDevice extends SubsystemBase {
  ShuffleboardSubsystem shuffle = ShuffleboardSubsystem.getInstance();
  private NetworkTable mainTable;
  public int mode;
  private static LimelightDevice instance = null;
  public int speakerMiddleTag;
  public int speakerSideTag;
  public int ampTag;

  public LimelightDevice(String networkTablesKey) {
    mainTable =
        NetworkTableInstance.getDefault()
            .getTable(networkTablesKey); // gets the network table with key

    mode = 0;
  }

  public void setLight(boolean on) { // toggles lights (true for on, false for off)
    mainTable.getEntry("ledMode").setNumber(on ? 3 : 1);
  }

  public boolean getLight() {
    double light = (double) mainTable.getEntry("lightMode").getNumber(3);
    return light != 1;
  }

  public void setMode(int selection) { // sets pipeline
    mainTable.getEntry("pipeline").setNumber(selection);
  }

  public boolean tagDetected() { // returns true if tag is detected
    double ttarget;
    try {
      ttarget = mainTable.getEntry("tv").getDouble(0);
    } catch (NullPointerException e) {
      System.out.println("ttarget ERROR, EXCEPTION: " + e);
      ttarget = 0;
    }
    return ttarget != 0;
  }

  public int getTagID() { // returns id of apriltag or -1 if no tag is detected.
    int tid;
    try {
      tid = (int) mainTable.getEntry("tid").getDouble(-1);
    } catch (NullPointerException e) {
      System.out.println("tid ERROR, EXCEPTION: " + e);
      tid = -1;
    }
    return tid;
  }

  public double getTagArea() { // return tag area
    double ta;
    try {
      ta = mainTable.getEntry("ta").getDouble(0);

    } catch (NullPointerException e) {
      System.out.println("Tag Area ERROR, EXCEPTION: " + e);
      ta = 0;
    }
    return ta;
  }

  public double getTagX() { // return tag x value (horizontal across camera screen)
    double tx;
    try {
      tx = mainTable.getEntry("tx").getDouble(0);
    } catch (NullPointerException e) {
      System.out.println("tx ERROR, EXCEPTION: " + e);
      tx = 0;
    }
    return tx;
  }

  public double getTagY() { // return tag y value (vertical across camera screen)
    double ty;
    try {
      ty = mainTable.getEntry("ty").getDouble(0);
    } catch (NullPointerException e) {
      System.out.println("ty ERROR, EXCEPTION: " + e);
      ty = 0;
    }

    return ty;
  }

  public void shuffleSetUp() {
    shuffle.setTab("Limelight Data");

    shuffle.addCamera("Limelight Stream", "Limelight", "10.72.0.11:5800");

    shuffle.setLayout("Limelight Config");
    shuffle.setBoolean("LEDs On", false);
    shuffle.setNumber("Pipeline", -1);

    shuffle.setLayout("Tag", 2, 4);
    shuffle.setBoolean("Tag Detected", false);
    shuffle.setNumber("Tag ID", -1);
    shuffle.setNumber("Tag X", -1);
    shuffle.setNumber("Tag Y", -1);
    shuffle.setNumber("Tag Area", -1);
    shuffle.setLayout(null);
  }

  public void shuffleUpdate() {
    double ledMode = mainTable.getEntry("ledMode").getDouble(0);
    double pipeline = mainTable.getEntry("pipeline").getDouble(0);
    double ttarget = mainTable.getEntry("tv").getDouble(0);
    double tid = mainTable.getEntry("tid").getDouble(-1);
    double tx = mainTable.getEntry("tx").getDouble(0);
    double ty = mainTable.getEntry("ty").getDouble(0);
    double ta = mainTable.getEntry("ta").getDouble(0);

    boolean ledOn = ledMode == 3 ? true : false;
    boolean tdetected = ttarget == 0 ? false : true;

    shuffle.setBoolean("LEDs On", ledOn);
    shuffle.setNumber("Pipeline", pipeline);
    shuffle.setBoolean("Tag Detected", tdetected);
    shuffle.setNumber("Tag ID", tid);
    shuffle.setNumber("Tag X", tx);
    shuffle.setNumber("Tag Y", ty);
    shuffle.setNumber("Tag Area", ta);
  }
}
