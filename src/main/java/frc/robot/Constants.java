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
      public static final double P = 0.007;
      public static final double I = 0.003;
      public static final double D = 0.002;
    }

    public class Positions {
      public static final double leftUpPosition = 239;
      public static final double leftDownPosition = 175;

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

    public class RightPID {
      public static final double P = 0.005;
      public static final double I = 0;
      public static final double D = 0.001;
    }

    public class LeftPID {
      public static final double P = 0.009;
      public static final double I = 0;
      public static final double D = 0.004;
    }

    public class Positions {
      public static final double leftMaxPosition = 270; // 217
      public static final double leftMinPosition = 222; // 171

      public static final double rightMaxPosition = 320; // 292
      public static final double rightMinPosition = 230; // 232

      public static final double upperStopRange = 2;
      public static final double lowerStopRange = 5;
    }

    public final class motorControllerConfigurations {
      public static final int currentLimit = 40;
    }
  }

  public final class DeviceIDs {
    public static final int leftLegMotor = 1;
    public static final int rightLegMotor = 2;
    public static final int leftArmMotor = 4;
    public static final int rightArmMotor = 3;
  }
}
