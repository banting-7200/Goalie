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
      public static final double P = 1.553;
      public static final double I = 0;
      public static final double D = 0.503;
    }

    public class Positions {
      public static final double upPosition = 0.230;
      public static final double downPosition = 0.342;
      public static final double stopRange = 0.05;
    }
  }

  public final class DeviceIDs {
    public static final int leftLegMotor = 1;
    public static final int rightLegMotor = 0;
  }
}
