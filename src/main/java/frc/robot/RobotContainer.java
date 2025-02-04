// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.event.BooleanEvent;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.*;
import frc.robot.Constants.Vision.lowerCamera;
import frc.robot.Constants.Vision.upperCamera;
import frc.robot.Subsystems.*;
import frc.robot.Vision.*;
import java.io.File;

public class RobotContainer {

  XboxController driveController = new XboxController(Constants.Control.Main.controllerPort);
  Joystick buttonBox = new Joystick(Constants.Control.Support.port);

  public LegSubsystem leftLeg;
  public LegSubsystem rightLeg;

  public ArmSubsystem leftArm;
  public ArmSubsystem rightArm;

  public HeadSubsystem head;

  public photonVisionCamera camera1;
  public photonVisionCamera camera2;

  public SwerveSubsystem drivebase;

  public DualCameraVelocityTracker velocityTracker;

  private int testMode = 0;

  private boolean canMakeSave = false;

  private boolean manualMode = true;

  private EventLoop manualLoop = new EventLoop();
  private EventLoop autoLoop = new EventLoop();
  private EventLoop enabledLoop = new EventLoop();

  private ShuffleboardSubsystem shuffle = ShuffleboardSubsystem.getInstance();

  private LightsSubsystem lights;

  Command driveFieldOrientedDirectAngle;

  public RobotContainer() {
    shuffle.setTab("Status");
    leftLeg =
        new LegSubsystem(
            DeviceIDs.leftLegMotor,
            Legs.Positions.leftDownPosition,
            Legs.Positions.leftUpPosition,
            false);

    leftLeg.setPID(Legs.leftPID.P, Legs.leftPID.I, Legs.leftPID.D);

    rightLeg =
        new LegSubsystem(
            DeviceIDs.rightLegMotor,
            Legs.Positions.rightDownPosition,
            Legs.Positions.rightUpPosition,
            true);

    rightLeg.setPID(Legs.rightPID.P, Legs.rightPID.I, Legs.rightPID.D);

    leftArm =
        new ArmSubsystem(
            DeviceIDs.leftArmMotor,
            Arms.Positions.leftMaxPosition,
            Arms.Positions.leftMinPosition,
            false);

    leftArm.setPID(Arms.LeftPID.P, Arms.LeftPID.I, Arms.LeftPID.D);

    rightArm =
        new ArmSubsystem(
            DeviceIDs.rightArmMotor,
            Arms.Positions.rightMaxPosition,
            Arms.Positions.rightMinPosition,
            true);

    rightArm.setPID(Arms.RightPID.P, Arms.RightPID.I, Arms.RightPID.D);

    camera1 = new photonVisionCamera(Constants.Vision.upperCamera.address);
    camera2 = new photonVisionCamera(Constants.Vision.lowerCamera.address);

    velocityTracker =
        new DualCameraVelocityTracker(
            camera1,
            upperCamera.xOffset,
            upperCamera.yOffset,
            camera2,
            lowerCamera.xOffset,
            lowerCamera.yOffset);

    drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve/neo"));

    driveFieldOrientedDirectAngle =
        drivebase.driveCommand(
            () -> MathUtil.applyDeadband(-driveController.getLeftY(), 0.1),
            () -> MathUtil.applyDeadband(-driveController.getLeftX(), 0.1),
            () -> -driveController.getRightX(),
            () -> -driveController.getRightY());

    head =
        new HeadSubsystem(
            Constants.DeviceIDs.headMotor,
            Constants.DeviceIDs.headLowerLimit,
            Constants.DeviceIDs.headUpperLimit);

    lights = new LightsSubsystem(Constants.DeviceIDs.lights, 59);

    configureBindings();
  }

  private void configureBindings() {
    shuffle.setPID("PID Tuner", Arms.RightPID.P, Arms.RightPID.I, Arms.RightPID.D);

    BooleanEvent toggleLeftLeg =
        new BooleanEvent(
            manualLoop, () -> driveController.getRawButton(Control.Main.leftLegToggleButton));
    toggleLeftLeg.rising().ifHigh(() -> leftLeg.togglePosition());

    BooleanEvent toggleRightLeg =
        new BooleanEvent(
            manualLoop, () -> driveController.getRawButton(Control.Main.rightLegToggleButton));
    toggleRightLeg.rising().ifHigh(() -> head.toggleHead());

    BooleanEvent toggleHead =
        new BooleanEvent(
            manualLoop, () -> driveController.getRawButton(Control.Main.toggleHeadButton));
    toggleHead.rising().ifHigh(() -> head.toggleHead());

    BooleanEvent zeroHead =
        new BooleanEvent(
            manualLoop, () -> driveController.getRawButton(Control.Main.zeroHeadButton));
    zeroHead.rising().ifHigh(() -> head.zeroEncoder());

    // Comment toggleRightArm if using joystick to control
    // BooleanEvent toggleRightArm = new BooleanEvent(manualLoop, () ->
    // driveController.getBButton());

    // toggleRightArm.rising().ifHigh(() -> rightArm.toggleArmPosition());
    // toggleRightArm.rising().ifHigh(() -> rightArm.toggleArmPosition());

    BooleanEvent toggleSafeMode =
        new BooleanEvent(manualLoop, () -> driveController.getRawButton(Control.Main.enableButton));

    toggleSafeMode
        .rising()
        .ifHigh(
            () -> {
              rightLeg.setEnabled(!rightLeg.isEnabled());
              leftLeg.setEnabled(!leftLeg.isEnabled());
              rightArm.setEnabled(!rightArm.isEnabled());
              leftArm.setEnabled(!leftArm.isEnabled());
              head.enableMovement(!head.isEnabled());
            });

    // BooleanEvent updatePIDs =
    //     new BooleanEvent(manualLoop, () ->
    // driveController.getRawButton(Control.Main.updatePIDsButton));

    // updatePIDs
    //     .rising()
    //     .ifHigh(
    //         () -> {
    //           double[] PID = shuffle.getPID("PID Tuner");
    //           // Simply change the below line to tune PIDs for another object.
    //           rightArm.setPID(PID);
    //           System.out.println("UPDATING PIDS");
    //         });

    BooleanEvent switchTestMode =
        new BooleanEvent(
            manualLoop, () -> driveController.getRawButton(Control.Main.switchTestModeButton));

    switchTestMode.rising().ifHigh(() -> testMode++);

    BooleanEvent clearCameraData =
        new BooleanEvent(
            manualLoop, () -> driveController.getRawButton(Control.Main.clearCameraDataButton));

    clearCameraData.rising().ifHigh(() -> velocityTracker.reset());

    BooleanEvent zeroGyro =
        new BooleanEvent(
            manualLoop, () -> driveController.getRawButton(Control.Main.zeroGyroButton));

    BooleanEvent resetBot =
        new BooleanEvent(
            autoLoop, () -> driveController.getRawButton(Control.Main.clearCameraDataButton));
    resetBot.rising().ifHigh(() -> reset());

    BooleanEvent toggleMode =
        new BooleanEvent(enabledLoop, () -> buttonBox.getRawButton(Control.Support.modeToggle));
    toggleMode.rising().ifHigh(() -> manualMode = !manualMode);

    zeroGyro.rising().ifHigh(() -> drivebase.zeroGyro());
    drivebase.setDefaultCommand(driveFieldOrientedDirectAngle);
  }

  public void periodic() {
    updateShuffle();
    updateTests();
  }

  public void enabledPeriodic() {
    lights.run();
    head.run();
    leftLeg.run();
    rightLeg.run();
    leftArm.run();
    rightArm.run();
    if (manualMode) {
      leftArm.moveFromRange(-1, 1, driveController.getLeftY());
      rightArm.moveFromRange(-1, 1, driveController.getLeftY());
      manualLoop.poll();
    } else {
      autoLoop.poll();
      makeSave();
    }
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
      case 5:
        estimateHitPoint();
        break;
      case 6:
        estimateSave();
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

  public void estimateHitPoint() {
    if (!camera1.hasTarget() || !camera2.hasTarget()) return;
    double[] hitPoint = velocityTracker.getHitPoint();
    System.out.println(
        String.format(
            "hitpoint: %d, %d, %d, in %d seconds",
            hitPoint[0], hitPoint[1], hitPoint[2], velocityTracker.getSecondsToImpact()));
  }

  public void estimateSave() {
    if (!camera1.hasTarget() || !camera2.hasTarget()) return;
    double secondsToImpact = velocityTracker.getSecondsToImpact();
    double[] hitPoint = velocityTracker.getHitPoint();
    if (hitPoint[0] > Constants.Robot.width / 2) {
      System.out.print("right");
    } else if (hitPoint[0] < -Constants.Robot.width / 2) {
      System.out.print("left");
    } else {
      System.out.print("middle");
    }

    if (hitPoint[1] > Constants.Robot.height / 2) {
      System.out.print(" top");
    } else if (hitPoint[1] < -Constants.Robot.height / 2) {
      System.out.print(" middle");
    } else {
      System.out.print(" bottom");
    }
    System.out.println(" in " + secondsToImpact + " seconds ");
  }

  public void reset() {
    canMakeSave = true;
    velocityTracker.reset();
    leftArm.moveToDownPosition();
    rightArm.moveToDownPosition();
    leftLeg.moveToUpPosition();
    rightLeg.moveToUpPosition();
  }

  public void makeSave() {
    if (!canMakeSave) return;
    double secondsToImpact = velocityTracker.getSecondsToImpact();
    double[] hitPoint = velocityTracker.getHitPoint();
    if (secondsToImpact > Constants.Robot.secondsBeforeSave) return;
    canMakeSave = false;
    if (hitPoint[1] > Constants.Robot.armActivationMaxHeight) // if too high do nothing
    return;
    if (hitPoint[1] > Constants.Robot.armActivationMinHeight) {
      if (hitPoint[0] > Constants.Robot.width / 2) {
        rightArm.moveFromRange(
            Constants.Robot.armActivationMinHeight,
            Constants.Robot.armActivationMaxHeight,
            hitPoint[1]);
      } else if (hitPoint[0] < -Constants.Robot.width / 2) {
        leftArm.moveFromRange(
            Constants.Robot.armActivationMinHeight,
            Constants.Robot.armActivationMaxHeight,
            hitPoint[1]);
      }
    } else {
      if (hitPoint[0] > Constants.Robot.width / 2) {
        rightLeg.moveToDownPosition();
      } else if (hitPoint[0] < -Constants.Robot.width / 2) {
        leftLeg.moveToDownPosition();
      }
    }
  }
}
