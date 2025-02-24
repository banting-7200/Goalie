// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.event.BooleanEvent;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Commands.DriveHorizontalCommand;
import frc.robot.Commands.EmptyCommand;
import frc.robot.Commands.NetAlignCommand;
import frc.robot.Constants.*;
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

  public PhotonVisionCamera upperCamera;
  public PhotonVisionCamera lowerCamera;
  public PhotonVisionCamera backCamera;

  public SwerveSubsystem drivebase;

  public PowerDistribution PDH = new PowerDistribution(20, ModuleType.kRev);

  public DualCameraVelocityTracker velocityTracker;

  private int testMode = 0;

  public boolean canMakeSave = false;

  private boolean manualMode = true;

  private EventLoop manualLoop = new EventLoop();
  private EventLoop autoLoop = new EventLoop();
  private EventLoop enabledLoop = new EventLoop();

  private ShuffleboardSubsystem shuffle = ShuffleboardSubsystem.getInstance();

  private LightsSubsystem lights;

  public double secondsBeforeSave;

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

    upperCamera = new PhotonVisionCamera(Constants.Vision.UpperCamera.address);
    lowerCamera = new PhotonVisionCamera(Constants.Vision.LowerCamera.address);
    backCamera = new PhotonVisionCamera(Constants.Vision.BackCamera.address);

    velocityTracker =
        new DualCameraVelocityTracker(
            upperCamera,
            Vision.UpperCamera.xOffset,
            Vision.UpperCamera.yOffset,
            Vision.UpperCamera.upTilt,
            Vision.UpperCamera.rightTilt,
            lowerCamera,
            Vision.LowerCamera.xOffset,
            Vision.LowerCamera.yOffset,
            Vision.LowerCamera.upTilt,
            Vision.LowerCamera.rightTilt);

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

    lights = new LightsSubsystem(Constants.DeviceIDs.lights, 2);

    configureBindings();
  }

  private void configureBindings() {
    shuffle.setPID("PID Tuner", Arms.RightPID.P, Arms.RightPID.I, Arms.RightPID.D);
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

    // ----------------------ButtonBox-----------------------------------

    BooleanEvent toggleLeftLeg =
        new BooleanEvent(
            manualLoop, () -> buttonBox.getRawButton(Control.Support.leftLegToggleButton));
    toggleLeftLeg.rising().ifHigh(() -> leftLeg.togglePosition());

    BooleanEvent toggleRightLeg =
        new BooleanEvent(
            manualLoop, () -> buttonBox.getRawButton(Control.Support.rightLegToggleButton));
    toggleRightLeg.rising().ifHigh(() -> rightLeg.togglePosition());

    BooleanEvent toggleHead =
        new BooleanEvent(
            manualLoop, () -> buttonBox.getRawButton(Control.Support.toggleHeadButton));
    toggleHead.rising().ifHigh(() -> head.toggleHead());

    BooleanEvent zeroHead =
        new BooleanEvent(manualLoop, () -> buttonBox.getRawButton(Control.Support.zeroHeadButton));
    zeroHead.rising().ifHigh(() -> head.zeroEncoder());

    BooleanEvent toggleSafeMode =
        new BooleanEvent(
            enabledLoop, () -> buttonBox.getRawButton(Control.Support.enableMotorsSwitch));

    BooleanEvent refreshScheduler =
        new BooleanEvent(enabledLoop, () -> buttonBox.getRawButton(Control.Support.waveButton));
    refreshScheduler.rising().ifHigh(() -> new EmptyCommand().schedule());

    toggleSafeMode
        .rising()
        .ifHigh(
            () -> {
              rightLeg.setEnabled(true);
              leftLeg.setEnabled(true);
              rightArm.setEnabled(true);
              leftArm.setEnabled(true);
              head.enableMovement(true);
            });
    toggleSafeMode
        .negate()
        .ifHigh(
            () -> {
              rightLeg.setEnabled(false);
              leftLeg.setEnabled(false);
              rightArm.setEnabled(false);
              leftArm.setEnabled(false);
              head.enableMovement(false);
            });

    BooleanEvent clearCameraData =
        new BooleanEvent(
            manualLoop, () -> buttonBox.getRawButton(Control.Support.clearCameraDataButton));

    clearCameraData.rising().ifHigh(() -> velocityTracker.reset());

    BooleanEvent resetBot =
        new BooleanEvent(
            autoLoop, () -> buttonBox.getRawButton(Control.Support.clearCameraDataButton));
    resetBot.rising().ifHigh(() -> reset());

    BooleanEvent setRobotMode =
        new BooleanEvent(
            enabledLoop, () -> buttonBox.getRawButton(Control.Support.manualModeSwitch));

    setRobotMode.rising().ifHigh(() -> setManualMode(false));
    setRobotMode.negate().rising().ifHigh(() -> setManualMode(true));

    // ----------------------XboxController--------------------------------

    BooleanEvent switchTestMode =
        new BooleanEvent(
            manualLoop, () -> driveController.getRawButton(Control.Main.switchTestModeButton));

    switchTestMode.rising().ifHigh(() -> testMode++);

    BooleanEvent zeroGyro =
        new BooleanEvent(
            enabledLoop, () -> driveController.getRawButton(Control.Main.zeroGyroButton));
    zeroGyro.rising().ifHigh(() -> drivebase.zeroGyro());
    drivebase.setDefaultCommand(driveFieldOrientedDirectAngle);
  }

  // --------------------------------------------------------------------------

  public void setManualMode(boolean manualMode) {
    canMakeSave = false;
    this.manualMode = manualMode;
  }

  public void periodic() {
    updateShuffle();
    // updateTests();
    // lights.rainbow();
    // lights.run();

  }

  public void criticalPeriodic() {
    if (!manualMode) {
      makeSave();
    }
    leftLeg.run();
    rightLeg.run();
    leftArm.run();
    rightArm.run();
    head.run();
  }

  public void enabledPeriodic() {
    if (manualMode) {
      leftArm.moveFromRange(-1, 1, buttonBox.getX());
      rightArm.moveFromRange(-1, 1, buttonBox.getY());
      manualLoop.poll();
    } else {
      autoLoop.poll();
    }
    head.run();
    enabledLoop.poll();
  }

  public void updateTests() {
    // if (testMode != 0) System.out.print(String.valueOf(testMode) + "|");
    switch (testMode) {
      case 0:
        break;
      case 1:
        upperCamera.test();
        break;
      case 2:
        lowerCamera.test();
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
        countFrames();
        break;
      case 7:
        // estimateSave();
        break;
      default:
        testMode = 0;
        break;
    }
  }

  public void updateShuffle() {
    shuffle.setTab("Goalia");

    shuffle.setLayout("Left Leg", 1, 3);
    shuffle.setBoolean("Left Leg Up", leftLeg.isUp());
    shuffle.setBoolean("Left Leg Locked", leftLeg.isEnabled());
    shuffle.setNumber("Left Leg Current", leftLeg.getCurrent());

    shuffle.setLayout("Right Leg", 1, 3);
    shuffle.setBoolean("Right Leg Up", rightLeg.isUp());
    shuffle.setBoolean("Right Leg Locked", rightLeg.isEnabled());
    shuffle.setNumber("Right Leg Current", rightLeg.getCurrent());

    shuffle.setLayout("Left Arm", 1, 3);
    shuffle.setNumber("Left Arm Position", leftArm.getPosition());
    shuffle.setBoolean("Left Arm Enabled", leftArm.isEnabled());
    shuffle.setNumber("Left Arm Current", leftArm.getCurrent());

    shuffle.setLayout("Right Arm", 1, 3);
    shuffle.setNumber("Right Arm Position", rightArm.getPosition());
    shuffle.setBoolean("Right Arm Enabled", rightArm.isEnabled());
    shuffle.setNumber("Right Arm Current", rightArm.getCurrent());

    shuffle.setLayout("Head", 1, 2);
    shuffle.setBoolean("Enabled", head.isEnabled());
    shuffle.setNumber("Position", head.getCurrentPosition());

    shuffle.setLayout("Status", 1, 2);
    shuffle.setBoolean("Auto", !manualMode);
    shuffle.setBoolean("Ready", canMakeSave);

    shuffle.setLayout("Cameras");
    shuffle.setBoolean("Upper Camera Has Target", upperCamera.hasTarget());
    shuffle.setBoolean("Lower Camera Has Target", lowerCamera.hasTarget());
    shuffle.setBoolean("Back Camera Has Target", backCamera.hasTarget());

    if (velocityTracker.hasTarget()) {
      shuffle.setLayout("Puck Pose", 1, 3);
      shuffle.setNumber("Puck X", velocityTracker.getTargetXPosition());
      shuffle.setNumber("Puck Y", velocityTracker.getTargetYPosition());
      shuffle.setNumber("Puck Distance", velocityTracker.getDistance());

      shuffle.setLayout("Puck Velocity", 1, 3);
      shuffle.setNumber("Puck Horizontal", velocityTracker.getRecentAverageHorizontalVelocity());
      shuffle.setNumber("Puck Vertical", velocityTracker.getAverageVerticalVelocity());
      shuffle.setNumber("Puck Incoming", velocityTracker.getAverageIncomingVelocity());

      shuffle.setLayout("Puck Hitpoint", 1, 3);
      shuffle.setNumber("Seconds To Impact", velocityTracker.getSecondsToImpact());
      double[] hitpoint = velocityTracker.getHitPoint();
      shuffle.setNumber("Hitpoint X", hitpoint[0]);
      shuffle.setNumber("Hitpoint Y", hitpoint[1]);
    }
  }

  public void estimateHitPoint() {
    if (!velocityTracker.hasTarget()) return;
    double[] hitPoint = velocityTracker.getHitPoint();
    if (hitPoint == new double[2]) {
      System.out.println("hasTarget");
    } else {
      System.out.println(
          String.format(
              "hitpoint: %.2f, %.2f, in %.2f seconds",
              hitPoint[0], hitPoint[1], velocityTracker.getSecondsToImpact()));
    }
  }

  public void reset() {
    new NetAlignCommand(drivebase, backCamera)
        .andThen(
            () -> {
              canMakeSave = true;
              velocityTracker.reset();
              leftArm.moveToDownPosition();
              rightArm.moveToDownPosition();
              leftLeg.moveToUpPosition();
              rightLeg.moveToUpPosition();
            })
        .schedule();
  }

  public void makeSave() {
    if (!canMakeSave) return; // ensure it is ready
    if (velocityTracker.hasTarget()) { // if cameras see the puck
      if (velocityTracker.getSecondsToImpact()
              < Constants.Robot.secondsBeforeSave // if puck going to
          && velocityTracker.getSecondsToImpact() > 0) {
        double saveTime = -Timer.getFPGATimestamp();
        canMakeSave = false;
        System.out.println("Tracker Latency: " + velocityTracker.getLatency());
        double[] hitPoint = velocityTracker.getHitPoint();
        if (hitPoint[1] > Constants.Robot.legActivationMaxHeight) { // if not legs
          if (hitPoint[1] < Constants.Robot.armActivationMaxHeight) { // if not above net
            // arms
            if (hitPoint[0] > Constants.Robot.width / 2) { // if on right
              if (hitPoint[1]
                  > Constants.Robot.rightArmActivationMinHeight) { // if within arm range on right
                rightArmSave(hitPoint[1]);
              } else { // if in between arm and leg on right
                rightMiddleSave();
              }
            } else if (hitPoint[0] < -Constants.Robot.width / 2) { // if on left
              if (hitPoint[1]
                  > Constants.Robot.leftArmActivationMinHeight) { // if within arm range on right
                leftArmSave(hitPoint[1]);
              } else {
                leftMiddleSave();
              }
            } else { // if in middle
              System.out.println("Torso");
            }
          } else { // if above net
            System.out.println("Too High");
          }
        } else { // if legs
          if (hitPoint[0] > Constants.Robot.width / 2) { // if on right
            rightLegSave();
          } else if (hitPoint[0] < -Constants.Robot.width / 2) { // if on left
            leftLegSave();
          } else { // if in middle
            middleLegSave();
          }
        }
        System.out.println(String.format("Hitpoint: %2f, %2f", hitPoint[0], hitPoint[1]));
        saveTime += Timer.getFPGATimestamp();
        System.out.println("Save Time: " + saveTime);
      } else {
        System.out.println("Has Target " + velocityTracker.getSecondsToImpact());
      }
    }
  }

  public void leftLegSave() {
    leftLeg.moveToDownPosition();
    rightLeg.moveToMidPosition();
    System.out.println("Left Leg");
    new DriveHorizontalCommand(drivebase, -Constants.Robot.SlideDistance).schedule();
  }

  public void rightLegSave() {
    rightLeg.moveToDownPosition();
    leftLeg.moveToMidPosition();
    System.out.println("Right Leg");
    new DriveHorizontalCommand(drivebase, Constants.Robot.SlideDistance).schedule();
  }

  public void leftArmSave(double height) {
    double armPercent =
        ((height - Constants.Robot.leftArmActivationMinHeight)
            / (Constants.Robot.leftArmActivationMaxHeight
                - Constants.Robot.leftArmActivationMinHeight));
    leftArm.moveFromRange(0, 0.8, armPercent);
    rightLeg.moveToMidPosition();
    System.out.println("Left Arm");
    new DriveHorizontalCommand(drivebase, -Constants.Robot.SlideDistance).schedule();
  }

  public void rightArmSave(double height) {
    double armPercent =
        ((height - Constants.Robot.rightArmActivationMinHeight)
            / (Constants.Robot.rightArmActivationMaxHeight
                - Constants.Robot.rightArmActivationMinHeight));
    rightArm.moveFromRange(0, 0.8, armPercent);
    leftLeg.moveToMidPosition();
    System.out.println("Right Arm");
    new DriveHorizontalCommand(drivebase, Constants.Robot.SlideDistance).schedule();
  }

  public void rightMiddleSave() {
    rightArm.moveToDownPosition();
    rightLeg.moveToUpPosition();
    leftLeg.moveToMidPosition();
    new DriveHorizontalCommand(drivebase, 1.5 * Constants.Robot.SlideDistance).schedule();
    System.out.println("Right Middle");
  }

  public void leftMiddleSave() {
    rightArm.moveToDownPosition();
    leftLeg.moveToUpPosition();
    rightLeg.moveToMidPosition();
    new DriveHorizontalCommand(drivebase, -1.5 * Constants.Robot.SlideDistance).schedule();
    System.out.println("Left Middle");
  }

  public void middleLegSave() {
    rightLeg.moveToDownPosition();
    leftLeg.moveToDownPosition();
    System.out.println("Both Legs");
  }

  // public void estimateSave() {
  //   if (!canMakeSave) return;
  //   if (velocityTracker.hasTarget()) {
  //     if (velocityTracker.getSecondsToImpact() < Constants.Robot.secondsBeforeSave
  //         && velocityTracker.getSecondsToImpact() > 0) {
  //       canMakeSave = false;
  //       double[] hitPoint = velocityTracker.getHitPoint();
  //       if (hitPoint[0] > Constants.Robot.width / 2) {
  //         System.out.print("right ");
  //       } else if (hitPoint[0] < -Constants.Robot.width / 2) {
  //         System.out.print("left ");
  //       } else {
  //         System.out.print("middle ");
  //       }
  //       if (hitPoint[1] > Constants.Robot.armActivationMinHeight) {
  //         if (hitPoint[1] < Constants.Robot.armActivationMaxHeight) {
  //           double armPercent =
  //               ((hitPoint[1] - Constants.Robot.armActivationMinHeight)
  //                   / (Constants.Robot.armActivationMaxHeight
  //                       - Constants.Robot.armActivationMinHeight));
  //           System.out.println("arm " + armPercent);
  //         } else {
  //           System.out.println("too high");
  //         }
  //       } else {
  //         System.out.println("leg");
  //       }
  //       System.out.println(String.format("Hitpoint: %2f, %2f", hitPoint[0], hitPoint[1]));
  //     } else {
  //       System.out.println("Has Target");
  //     }
  //   }
  // }

  public void countFrames() {
    if (velocityTracker.hasTarget()) {
      if (velocityTracker.getSecondsToImpact() < Constants.Robot.secondsBeforeSave
          && velocityTracker.getSecondsToImpact() > 0) {
        System.out.println(velocityTracker.getSecondsToImpact());
      } else {
        System.out.println("hasTarget");
      }
    }
  }
}
