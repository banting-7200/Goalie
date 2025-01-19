// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.event.BooleanEvent;
import edu.wpi.first.wpilibj.event.EventLoop;
import frc.robot.Constants.*;
import frc.robot.Subsystems.*;
import frc.robot.Subsystems.photonVisionCamera;

public class RobotContainer {

  XboxController controller = new XboxController(Constants.Controller.port);

  // Physical Components //
  public LegSubsystem leftLeg;
  public LegSubsystem rightLeg;
  public ArmSubsystem leftArm;
  public ArmSubsystem rightArm;
  public DualCameraVelocityTracker velocityTracker;
  public photonVisionCamera camera1;
  public photonVisionCamera camera2;
  public CANSparkMax IRLight;

  public int testMode = 0;
  private EventLoop loop = new EventLoop();
  private ShuffleboardSubsystem shuffle = ShuffleboardSubsystem.getInstance();

  public RobotContainer() {
    shuffle.setTab("Status");
    leftLeg =
        new LegSubsystem(
            DeviceIDs.leftLegMotor,
            Legs.Positions.leftDownPosition,
            Legs.Positions.leftUpPosition,
            false);

    rightLeg =
        new LegSubsystem(
            DeviceIDs.rightLegMotor,
            Legs.Positions.rightDownPosition,
            Legs.Positions.rightUpPosition,
            true);

    leftArm =
        new ArmSubsystem(
            DeviceIDs.leftArmMotor,
            Arms.Positions.leftMaxPosition,
            Arms.Positions.leftMinPosition,
            false,
            false);

    leftArm.setPID(Arms.LeftPID.P, Arms.LeftPID.I, Arms.LeftPID.D);

    rightArm =
        new ArmSubsystem(
            DeviceIDs.rightArmMotor,
            Arms.Positions.rightMaxPosition,
            Arms.Positions.rightMinPosition,
            true,
            false);

    rightArm.setPID(Arms.RightPID.P, Arms.RightPID.I, Arms.RightPID.D);

    IRLight = new CANSparkMax(5, MotorType.kBrushed);
    IRLight.setInverted(true);

    camera1 = new photonVisionCamera("Arducam_OV9281_USB_Camera");
    camera2 = new photonVisionCamera("Arducam_OV9281_USB_Camera (1)");
    velocityTracker = new DualCameraVelocityTracker(camera1, -0.07, 0.04, camera2, -0.07, -0.05);

    configureBindings();
  }

  private void configureBindings() {
    shuffle.setPID("PID Tuner", Arms.RightPID.P, Arms.RightPID.I, Arms.RightPID.D);

    BooleanEvent toggleLeftLeg =
        new BooleanEvent(
            loop, () -> controller.getRawButton(Controls.XboxController.leftLegToggleButton));
    toggleLeftLeg.rising().ifHigh(() -> leftLeg.togglePosition());

    BooleanEvent toggleRightLeg =
        new BooleanEvent(
            loop, () -> controller.getRawButton(Controls.XboxController.rightLegToggleButton));
    toggleRightLeg.rising().ifHigh(() -> rightLeg.togglePosition());

    // Comment toggleRightArm if using joystick to control
    BooleanEvent toggleRightArm = new BooleanEvent(loop, () -> controller.getBButton());

    toggleRightArm.rising().ifHigh(() -> rightArm.toggleArmPosition());

    BooleanEvent toggleSafeMode =
        new BooleanEvent(loop, () -> controller.getRawButton(Controls.XboxController.enableButton));

    toggleSafeMode
        .rising()
        .ifHigh(
            () -> {
              rightLeg.setEnabled(!rightLeg.isEnabled());
              leftLeg.setEnabled(!leftLeg.isEnabled());
              rightArm.setEnabled(!rightArm.isEnabled());
              leftArm.setEnabled(!leftArm.isEnabled());
            });

    BooleanEvent updatePIDs =
        new BooleanEvent(
            loop, () -> controller.getRawButton(Controls.XboxController.updatePIDsButton));

    updatePIDs
        .rising()
        .ifHigh(
            () -> {
              double[] PID = shuffle.getPID("PID Tuner");
              // Simply change the below line to tune PIDs for another object.
              rightArm.setPID(PID);
              System.out.println("UPDATING PIDS");
            });

    BooleanEvent switchTestMode =
        new BooleanEvent(
            loop, () -> controller.getRawButton(Controls.XboxController.switchTestModeButton));

    switchTestMode.rising().ifHigh(() -> testMode++);

    BooleanEvent clearCameraData =
        new BooleanEvent(
            loop, () -> controller.getRawButton(Controls.XboxController.clearCameraDataButton));

    clearCameraData.rising().ifHigh(() -> velocityTracker.reset());
  }

  public void periodic() {
    updateShuffle();
    updateTests();
    IRLight.set(0.5);
  }

  public void enabledPeriodic() {
    loop.poll();
    // leftLeg.run();
    // rightLeg.run();
    // leftArm.run();
    // rightArm.run();
  }

  public void updateTests() {
    if (testMode != 0) System.out.print(String.valueOf(testMode) + "|");
    switch (testMode) {
      case 0:
        break;
      case 1:
        camera1.test();
        break;
      case 2:
        camera2.test();
        break;
      case 3:
        velocityTracker.stationaryTest();
        break;
      case 4:
        velocityTracker.trajectoryTest();
        break;
      default:
        testMode = 0;
        break;
    }
  }

  public void updateShuffle() {
    shuffle.setTab("Status");

    shuffle.setLayout("Left Leg", 1, 2);
    shuffle.setBoolean("Left Leg Up", leftLeg.isUp());
    shuffle.setBoolean("Left Leg Locked", leftLeg.isEnabled());

    shuffle.setLayout("Right Leg", 1, 2);
    shuffle.setBoolean("Right Leg Up", rightLeg.isUp());
    shuffle.setBoolean("Right Leg Locked", rightLeg.isEnabled());

    shuffle.setLayout("Left Arm", 1, 2);
    shuffle.setNumber("Left Arm Position", leftArm.getPosition());
    shuffle.setBoolean("Left Arm Enabled", leftArm.isEnabled());
    shuffle.setNumber("Left Arm Current", leftArm.getCurrent());

    shuffle.setLayout("Right Arm", 1, 2);
    shuffle.setNumber("Right Arm Position", rightArm.getPosition());
    shuffle.setBoolean("Right Arm Enabled", rightArm.isEnabled());
    shuffle.setNumber("Right Arm Current", rightArm.getCurrent());
  }

  // public void estimateSave() {
  //   double secondsToImpact = velocityTracker.getSecondsToImpact();
  //   double[] hitPoint = velocityTracker.getHitPoint();

  //   if (hitPoint[0] > Constants.Robot.width / 2) {
  //     System.out.print("right");
  //   } else if (hitPoint[0] < -Constants.Robot.width / 2) {
  //     System.out.print("left");
  //   } else {
  //     System.out.print("middle");
  //   }

  //   if (hitPoint[1] > Constants.Robot.height / 2) {
  //     System.out.print(" top");
  //   } else if (hitPoint[1] < -Constants.Robot.height / 2) {
  //     System.out.print(" middle");
  //   } else {
  //     System.out.print(" bottom");
  //   }
  //   System.out.println(" in " + secondsToImpact + " seconds ");
  // }

  public void makeSave() {
    double box =
        1; // box where the puck is going, 1-6, 3 across, 2 down, starting at the top left from the
    // perspective of the robot
    // double secondsToImpact = velocityTracker.getSecondsToImpact();
    // double[] hitPoint = velocityTracker.getHitPoint();

    // if (hitPoint[0] > -Constants.Robot.width / 2) box++;
    // if (hitPoint[0] > Constants.Robot.width / 2) box++;
    // if (hitPoint[1] < 0) box += 3;
  }
}
