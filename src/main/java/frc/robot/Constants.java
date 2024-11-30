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

  }

  public final class Legs {
    public class PID {
      public static double P = 0.007;
      public static double I = 0.003;
      public static double D = 0.002;
    }

    public class Positions {
      public static final double leftUpPosition = 239.5;
      public static final double leftDownPosition = 176;

      public static final double rightUpPosition = 130;
      public static final double rightDownPosition = 52;

      public static final double upperStopRange = 1;
      public static final double lowerStopRange = 5;
    }

    public final class motorControllerConfigurations {
      public static final int currentLimit = 40;
    }
  }
  
  public final class Arms {

    public class PID {
      public static double P = 0;
      public static double I = 0;
      public static double D = 0;
    }

    public class Positions {
      public static final double leftUpPosition = 0;
      public static final double leftDownPosition = 0;

      public static final double rightUpPosition = 0;
      public static final double rightDownPosition = 0;

      public static final double stopRange = 2;
    }

    public final class motorControllerConfigurations {
      public static final int currentLimit = 40;
    }
  }

  public final class DeviceIDs {
    public static final int leftLegMotor = 1;
    public static final int rightLegMotor = 2;
  }

  
}
