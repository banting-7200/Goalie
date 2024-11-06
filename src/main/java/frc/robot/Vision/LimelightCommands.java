package frc.robot.Vision;

public class LimelightCommands {
  // Creates a instance of the LimelightDevice class, instance ensures only 1 instance of
  // limelightdevice is created
  LimelightDevice limelight = LimelightDevice.getInstance();
  private double tagX;
  private double tagY;
  private double tagArea;
  private double tagID;

  public void readAprilTag() {
    // Read all basic data from the limelight and assign them to local variables
    this.tagX = limelight.getTagX();
    this.tagY = limelight.getTagY();
    this.tagArea = limelight.getTagArea();
    this.tagID = limelight.getTagID();
  }

  public void moveToAprilTagOffset(double distanceFromTag, double positionOffset) {
    // Expects parameters of distance from the april tag and the position offset
    // Offset is linked to robot position, ex.. -1 would be left of tag while 1 would be right of
    // the tag

  }

  public void followAprilTag(double distanceFromTag) {
    // Basic follow april tag method, centers with april tag, and follows from set distance

  }
}
