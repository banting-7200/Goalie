package frc.robot.Subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.ArrayList;

public class DualCameraVelocityTracker extends SubsystemBase {

  private final photonVisionCamera c1;
  private final photonVisionCamera c2;

  private final double c1xOffset;
  private final double c1yOffset;

  private final double c2xOffset;
  private final double c2yOffset;

  private final double frameRate = 144;

  private ArrayList<double[]> positions = new ArrayList<>();
  private ArrayList<double[]> velocities = new ArrayList<>();

  /**
   * Initializes a dual camera velocity tracker object.
   *
   * @param camera1 the highest, leftmost camera
   * @param camera1xOffset the x offset of this camera from the centre of the robot in metres
   * @param camera1yOffset the y offset of this camera from the centre of the robot in metres
   * @param camera2 the lowest, rightmost camera
   * @param camera2xOffset the x offset of this camera from the centre of the robot in metres
   * @param camera2yOffset the y offset of this camera from the centre of the robot in metres
   */
  public DualCameraVelocityTracker(
      photonVisionCamera camera1,
      double camera1xOffset,
      double camera1yOffset,
      photonVisionCamera camera2,
      double camera2xOffset,
      double camera2yOffset) {
    this.c1 = camera1;
    this.c2 = camera2;
    this.c1xOffset = camera1xOffset;
    this.c1yOffset = camera1yOffset;
    this.c2xOffset = camera2xOffset;
    this.c2yOffset = camera2yOffset;
  }

  @Override
  public void periodic() {
    if (c1.hasTarget() && c2.hasTarget()) {
      addRecentPosition();
      addRecentVelocity();
    }
  }

  public double getDistance() {
    double horizontalGap = Math.abs(c1xOffset - c2xOffset);

    double yaw1 = Math.toRadians(Math.abs(90 - c2.getTargetYaw()));
    double yaw2 = Math.toRadians(Math.abs(90 + c1.getTargetYaw()));
    double oppAngle = Math.PI - yaw1 - yaw2;

    double side1 = Math.sin(yaw2) * horizontalGap / Math.sin(oppAngle);
    double side2 = Math.sin(yaw1) * horizontalGap / Math.sin(oppAngle);

    double semiPerimeter = (side1 + side2 + horizontalGap) / 2;

    double area =
        Math.sqrt(
            semiPerimeter
                * (semiPerimeter - horizontalGap)
                * (semiPerimeter - side1)
                * (semiPerimeter - side2));

    double hDistanceEstimate = area * 2 / horizontalGap;

    double verticalGap = Math.abs(c1yOffset - c2yOffset);

    double pitch1 = Math.toRadians(Math.abs(90 + c1.getTargetYaw()));
    double pitch2 = Math.toRadians(Math.abs(90 - c2.getTargetYaw()));
    double oppPitch = Math.PI - pitch1 - pitch2;

    double pSide1 = Math.sin(pitch2) * verticalGap / Math.sin(oppPitch);
    double pSide2 = Math.sin(pitch1) * verticalGap / Math.sin(oppPitch);

    double pSemiPerimeter = (pSide1 + pSide2 + verticalGap) / 2;

    double pArea =
        Math.sqrt(
            pSemiPerimeter
                * (pSemiPerimeter - verticalGap)
                * (pSemiPerimeter - pSide1)
                * (pSemiPerimeter - pSide2));

    double vDistanceEstimate = pArea * 2 / verticalGap;
    return (hDistanceEstimate + vDistanceEstimate) / 2;
  }

  public double getTargetXPosition() {
    return getDistance() / Math.tan(Math.toRadians(90 - c1.getTargetYaw()));
  }

  public double getTargetYPosition() {
    return getDistance() / Math.tan(Math.toRadians(90 - c1.getTargetPitch()));
  }

  public void addPosition(double x, double y, double z) {
    double[] position = {x, y, z};
    positions.add(position);
  }

  public void addRecentPosition() {
    addPosition(getTargetXPosition(), getTargetYPosition(), getDistance());
  }

  public Double getRecentHorizontalVelocity() {
    if (positions.size() < 2) return null;
    double position1 = positions.get(positions.size() - 2)[0];
    double position2 = positions.get(positions.size() - 1)[0];
    double velocity = (position2 - position1) * frameRate;
    return velocity;
  }

  public Double getRecentVerticalVelocity() {
    if (positions.size() < 2) return null;
    double position1 = positions.get(positions.size() - 2)[1];
    double position2 = positions.get(positions.size() - 1)[1];
    double velocity = (position2 - position1) * frameRate;
    return velocity;
  }

  public Double getRecentIncomingVelocity() {
    if (positions.size() < 2) return null;
    double position1 = positions.get(positions.size() - 2)[2];
    double position2 = positions.get(positions.size() - 1)[2];
    double velocity = (position1 - position2) * frameRate;
    return velocity;
  }

  public void addRecentVelocity() {
    if (positions.size() < 2) return;
    double[] velocity = {
      getRecentHorizontalVelocity(), getRecentVerticalVelocity(), getRecentIncomingVelocity()
    };
    velocities.add(velocity);
  }

  public double getAverageHorizontalVelocity() {
    double avgVelocity = 0;
    for (double[] velocity : velocities) {
      avgVelocity += velocity[0];
    }
    avgVelocity /= velocities.size();
    return avgVelocity;
  }

  public double getAverageVerticalVelocity() {
    double avgVelocity = 0;
    for (double[] velocity : velocities) {
      avgVelocity += velocity[1];
    }
    avgVelocity /= velocities.size();
    return avgVelocity;
  }

  public double getAverageIncomingVelocity() {
    double avgVelocity = 0;
    for (double[] velocity : velocities) {
      avgVelocity += velocity[2];
    }
    avgVelocity /= velocities.size();
    return avgVelocity;
  }

  public double getSecondsToImpact() {
    return getDistance() / getAverageIncomingVelocity();
  }

  public double[] getHitPoint() {
    double secondsToImpact = getSecondsToImpact();
    double xPosition = getTargetXPosition() + getAverageHorizontalVelocity() * secondsToImpact;
    double yPosition = getTargetYPosition() + getAverageVerticalVelocity() * secondsToImpact;
    return new double[] {xPosition, yPosition};
  }

  /**
   * Prints test data about the position of the target based on the current frame, regardless of
   * motion. - distance in metres - position right in metres - position up in metres
   */
  public void stationaryTest() {
    if (!c1.hasTarget() || !c2.hasTarget()) return;
    System.out.println(
        "Distance:"
            + String.format("%.2f", getDistance())
            + " Right:"
            + String.format("%.2f", getTargetXPosition())
            + " Up:"
            + String.format("%.2f", getTargetYPosition()));
  }

  /**
   * Prints test data about the velocity and trajectory of the target since the previous reset. -
   * incoming velocity in metres per second - rightward velocity in metres per second - upward
   * velocity in metres per second
   */
  public void trajectoryTest() {
    if (!c1.hasTarget() || !c2.hasTarget()) return;
    System.out.println(
        "Incoming:"
            + String.format("%.2f", getAverageIncomingVelocity())
            + " Horizontal:"
            + String.format("%.2f", getAverageHorizontalVelocity())
            + " Vertical:"
            + String.format("%.2f", getAverageVerticalVelocity()));
  }

  public void reset() {
    positions.clear();
    velocities.clear();
  }
}
