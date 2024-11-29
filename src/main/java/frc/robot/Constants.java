// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public final class Controller {
    public static final int port = 0;

    public final class Buttons {
      public static final int toggleLegs = 1;
    }
  }

  public final class Legs {
    public class PID {
      public static double P = 0.007;
      public static double I = 0.003;
      public static double D = 0.002;
    }

    public class Positions {
      public static final double upPosition = 122; // 240
      public static final double downPosition = 50; // 174
      public static final double stopRange = 3;
    }

    public class motorSpeeds {
      public static final double maxRPM = 5700;
      public static final double motorAccel = 0.1;
      public static final double allowedError = 0.009;
      public static double fastMaxVel = 0.1;
      public static double fastMinVel = 0.1;
      public static double slowMaxVel = 0.1;
      public static double slowMinVel = 0.1;
      public static double targetRPM = 2000;
    }
  }

  public final class DeviceIDs {
    public static final int leftLegMotor = 1;
    public static final int rightLegMotor = 2;
  }
}
