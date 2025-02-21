// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.XboxController;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public final class Robot {
    public static final double width = 0.1; // metres
    // public static final double height = .45; // metres
    public static final double armActivationMinHeight =
        0.40; // metres from position on robot camera positions are based on
    public static final double armActivationMaxHeight =
        1.10; // metres from position on robot camera positions are based on
    public static final double SlideDistance = 0.2;
    public static final double GravityEffect = 0.1;
    public static final double secondsBeforeSave = 2.0;
  }

  public final class Legs {
    public class leftPID {
      public static final double P = 0.009;
      public static final double I = 0.003;
      public static final double D = 0.002;
    }

    public class rightPID {
      public static final double P = 0.009;
      public static final double I = 0.003;
      public static final double D = 0.002;
    }

    public class Positions {
      public static final double leftUpPosition = 119;
      public static final double leftDownPosition = 44.5;

      public static final double rightUpPosition = 163.3;
      public static final double rightDownPosition = 93.5;

      public static final double upperStopRange = 0;
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
      public static final double P = 0.035;
      public static final double I = 0;
      public static final double D = 0.004;
    }

    public class Positions {
      public static final double leftMaxPosition = 185; // 217
      public static final double leftMinPosition = 140; // 171

      public static final double rightMaxPosition = 179; // 292
      public static final double rightMinPosition = 135; // 232

      public static final double upperStopRange = 1;
      public static final double lowerStopRange = 5;
    }

    public final class motorControllerConfigurations {
      public static final int currentLimit = 20;
    }
  }

  public final class DeviceIDs {
    public static final int leftLegMotor = 3;
    public static final int rightLegMotor = 4;
    public static final int leftArmMotor = 5;
    public static final int rightArmMotor = 6;
    public static final int headMotor = 7;
    public static final int headLowerLimit = 0;
    public static final int headUpperLimit = 1;
    public static final int lights = 0;
  }

  public final class Vision {
    public final class UpperCamera {
      public static final String address = "UpperFrontCamera";
      public static final double xOffset = 0;
      public static final double yOffset = 0.89;
      public static final double upTilt = -5;
      public static final double rightTilt = -3;
    }

    public final class LowerCamera {
      public static final String address = "LowerFrontCamera";
      public static final double xOffset = 0;
      public static final double yOffset = -.13;
      public static final double upTilt = 15;
      public static final double rightTilt = 0;
    }

    public final class BackCamera {
      public static final String address = "BackCamera";
    }
  }

  public final class Control {
    public final class Main {
      public static final int controllerPort = 0;
      public static final int zeroGyroButton = XboxController.Button.kA.value;
      public static final int leftLegToggleButton = XboxController.Button.kX.value;
      public static final int rightLegToggleButton = XboxController.Button.kB.value;
      public static final int enableButton = XboxController.Button.kY.value;
      public static final int leftArmChannel = XboxController.Axis.kLeftY.value;
      public static final int rightArmChannel = XboxController.Axis.kRightY.value;
      public static final int clearCameraDataButton = XboxController.Button.kLeftBumper.value;
      public static final int switchTestModeButton = XboxController.Button.kRightBumper.value;
      public static final int toggleHeadButton = 8;
      public static final int zeroHeadButton = XboxController.Button.kBack.value;
    }

    public final class Support {
      public static final int port = 1;
      public static final int modeToggle = 2;
    }
  }

  public final class Head {
    public class PID {
      public static final double P = 0.1;
      public static final double I = 0;
      public static final double D = 0.5;
    }

    public final class Positions {
      public static final double maxPosition = 30000;
      public static final double minPosition = 2000;
    }
  }

  public final class Drivebase {
    public static final double maxSpeed = Units.feetToMeters(1);

    public final class TranslationPID {
      public static final double p = 0.7;
      public static final double i = 0;
      public static final double d = 0;
    }

    public final class RotationPID {
      public static final double p = 0.4;
      public static final double i = 0;
      public static final double d = 0.01;
    }
  }
}
