// package frc.robot.Subsystems;

// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import java.util.ArrayList;
// import java.util.Arrays;

// public class VelocityTracker extends SubsystemBase {
//   ArrayList<CameraData> dataList;

//   public VelocityTracker() {
//     dataList = new ArrayList<>();
//   }

//   public class CameraData {
//     private final double[] areas = new double[2];
//     private final double[] distances = new double[2];
//     private final double[] yaws = new double[2];
//     private final double[] pitches = new double[2];

//     private final ArrayList<Double> speeds = new ArrayList<>();
//     private final ArrayList<Double> horizontalAngles = new ArrayList<>();
//     private final ArrayList<Double> verticalAngles = new ArrayList<>();

//     private long prevTime;
//     private long currTime = System.currentTimeMillis();

//     private final Camera camera;
//     private final double referenceDistance;
//     private final double referenceArea;
//     private final double xOffset;
//     private final double yOffset;

//     public CameraData(
//         Camera camera,
//         double referenceArea,
//         double referenceDistance,
//         double xOffset,
//         double yOffset) {
//       this.camera = camera;
//       this.referenceArea = referenceArea;
//       this.referenceDistance = referenceDistance;
//       this.xOffset = xOffset;
//       this.yOffset = yOffset;
//     }

//     private void clearData() {
//       horizontalAngles.clear();
//       verticalAngles.clear();
//       speeds.clear();
//       Arrays.fill(distances, 0);
//       Arrays.fill(yaws, 0);
//       Arrays.fill(pitches, 0);
//       Arrays.fill(areas, 0);
//     }

//     private void addDistance(double distance) {
//       distances[1] = distances[0];
//       distances[0] = distance;
//     }

//     private void addHorizontalAngle(double angle) {
//       if (!Double.isNaN(angle)) horizontalAngles.add(angle);
//     }

//     private void addVerticalAngle(double angle) {
//       if (!Double.isNaN(angle)) verticalAngles.add(angle);
//     }

//     private void addYaw(double yaw) {
//       if (!Double.isNaN(yaw)) {
//         yaws[1] = yaws[0];
//         yaws[0] = yaw;
//       }
//     }

//     private void addPitch(double pitch) {
//       if (!Double.isNaN(pitch)) {
//         pitches[1] = pitches[0];
//         pitches[0] = pitch;
//       }
//     }

//     private void addArea(double area) {
//       if (!Double.isNaN(area)) {
//         areas[1] = areas[0];
//         areas[0] = area;
//       }
//     }

//     private void addSpeed(double speed) {
//       if(!Double.isNaN(speed))
//         speeds.add(speed);
//     }
//   }

//   public void addCamera(
//       Camera camera,
//       double referenceArea,
//       double referenceDistance,
//       double xOffset,
//       double yOffset) {
//     dataList.add(
//         new CameraData(camera, referenceArea, referenceDistance, xOffset, yOffset));
//   }

//   public CameraData getCameraData(Camera camera) {
//     for (CameraData data : dataList) {
//       if (data.camera != camera)
//         continue;
//       return data;
//     }
//     return null;
//   }

//   private double findLongestDiagonal(Camera camera) {
//     double width = camera.getTargetWidth();
//     double height = camera.getTargetHeight();
//     double diagonal =
//   }

//   private double findDistance(Camera camera) {
//     CameraData data = getCameraData(camera);
//     double distanceEstimate = data.referenceArea/data.referenceDistance
//   }

//   private double getRecentDistance() {
//     for (CameraData data : dataList) {
//       double distance = data.areas[0];
//     }

//     double distanceEstimate = 1 / (area * 37.2742) + 0.839629;
//     // (area - nearArea) / (farArea - nearArea) * (farDistance - nearDistance) + nearDistance;
//     // nearDistance + (farDistance - nearDistance) * ((area - nearArea) / (farArea - nearArea));

//     return distanceEstimate;
//   }

//   private double getRecentSpeed(Camera camera) {
//     // Data Source //
//     CameraData data = getCameraData(camera);
//     // Extract Data //
//     double distance1 = data.distances[1];
//     double distance2 = data.distances[0];
//     // Calculations //
//     double speed = 1000 * (distance1 - distance2) / (data.currTime - data.prevTime);
//     // Return //
//     return speed;
//   }

//   private double getRecentSpeed(){

//   }

//   public double getRecentHorizontalAngle(Camera camera) {
//     // Data Source //
//     CameraData data = getCameraData(camera);
//     // Extract Data //
//     double distance1 = data.distances[1];
//     double distance2 = data.distances[0];

//     double yaw1 = data.yaws[1];
//     double yaw2 = data.yaws[0];
//     // Calculations //

//     double alpha = Math.abs(yaw1 - yaw2);
//     double directionFactor = (yaw2 - yaw1) / Math.abs(yaw2 - yaw1);
//     double angle =
//         directionFactor
//             * (Math.abs(yaw1)
//                 + Math.sqrt(distance1 * (distance1 - (2 * distance2 * Math.cos(alpha)))));
//     return angle;
//   }

//   public double getRecentVerticalAngle(Camera camera) {
//     // Data Source //
//     CameraData data = getCameraData(camera);
//     // Extract Data //
//     double distance1 = data.distances[1];
//     double distance2 = data.distances[0];

//     double pitch1 = data.pitches[1];
//     double pitch2 = data.pitches[0];
//     // Calculations //

//     double alpha = Math.abs(pitch1 - pitch2);
//     double directionFactor = (pitch2 - pitch1) / Math.abs(pitch2 - pitch1);
//     double angle =
//         directionFactor
//             * (Math.abs(pitch1)
//                 + Math.sqrt(distance1 * (distance1 - (2 * distance2 * Math.cos(alpha)))));
//     return angle;
//   }

//   private double getAverageSpeed() {
//     double avgSpeed = 0;
//     double speedCount = 0;
//     for (CameraData data : dataList) {
//       for (double speed : data.speeds) {
//         avgSpeed += speed;
//       }
//       speedCount += data.speeds.size();
//     }
//     avgSpeed /= speedCount;
//     return avgSpeed;
//   }

//   private double getAverageHorizontalAngle() {
//     double avgAngle = 0;
//     double angleCount = 0;
//     for (CameraData data : dataList) {
//       for (double angle : data.horizontalAngles) {
//         avgAngle += angle;
//       }
//       angleCount += data.horizontalAngles.size();
//     }
//     avgAngle /= angleCount;
//     return avgAngle;
//   }

//   private double getAverageVerticalAngle() {
//     double avgAngle = 0;
//     double angleCount = 0;
//     for (CameraData data : dataList) {
//       for (double angle : data.verticalAngles) {
//         avgAngle += angle;
//       }
//       angleCount += data.verticalAngles.size();
//     }
//     avgAngle /= angleCount;
//     return avgAngle;
//   }

//   // Overrides SubsystemBase Methods //
//   @Override
//   public void periodic() {
//     // Loop Through //
//     for (CameraData data : dataList) {
//       // Devices //
//       Camera camera = data.camera;
//       // Data //
//       if (camera.hasTarget()) {
//         data.prevTime = data.currTime;
//         data.currTime = System.currentTimeMillis();

//         data.addArea(camera.getTargetArea());
//         data.addYaw(camera.getTargetYaw());
//         data.addPitch(camera.getTargetPitch());
//         data.addDistance(getRecentDistance(camera));

//         if (data.distances[1] != 0) {
//           data.addSpeed(getRecentSpeed(camera));
//           data.addHorizontalAngle(getRecentHorizontalAngle(camera));
//           data.addVerticalAngle(getRecentVerticalAngle(camera));
//         }
//       }
//     }
//   }

//   public int getQuadrant(Camera camera) {
//     CameraData data = getCameraData(camera);
//     int quadrant = 1;
//     double xOffset = data.xOffset;
//     double yOffset = data.yOffset;

//     // double yaw = data.yaws[0];
//     // double incHor = getAverageHorizontalAngle() + yaw;
//     // double horizontalEndpoint = getRecentDistance(camera);
//     // horizontalEndpoint *= Math.sin(Math.toRadians(incHor + yaw));
//     // horizontalEndpoint /= Math.sin(Math.toRadians(180 - incHor));
//     // horizontalEndpoint += xOffset;

//     // double pitch = data.pitches[0];
//     // double incVer = getAverageVerticalAngle() + pitch;
//     // double verticalEndpoint = getRecentDistance(camera);
//     // verticalEndpoint *= Math.sin(Math.toRadians(incVer + pitch));
//     // verticalEndpoint /= Math.sin(Math.toRadians(180 - incVer));
//     // verticalEndpoint += yOffset;

//     // int quadrant = 0;
//     // if (horizontalEndpoint > 0) quadrant = 1;
//     // else quadrant = 2;
//     // if (verticalEndpoint < 0) quadrant += 2;
//     // return quadrant;

//     // goalies perspective
//     if (getAverageVerticalAngle() < 0) quadrant += 2;
//     if (getAverageHorizontalAngle() > 0) quadrant += 1;
//     return quadrant;
//   }

//   public void printData(Camera camera) {
//     if (camera.hasTarget()) {
//       System.out.println(
//           // "Vertical Angle: "
//           // + getAverageVerticalAngle()
//           // + "degrees, Horizontal Angle: "
//           // + getAverageHorizontalAngle()
//           // + "degrees, Speed: "
//           // + getAverageSpeed()
//           // + "m/s, quadrant: "
//           // + getQuadrant(camera));
//           "q: " + getQuadrant(camera));
//     }
//   }

//   public void clearData() {
//     for (CameraData data : dataList) {
//       data.clearData();
//     }
//   }
// }
