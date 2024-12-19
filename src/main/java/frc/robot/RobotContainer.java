// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.event.BooleanEvent;
import edu.wpi.first.wpilibj.event.EventLoop;
import frc.robot.Constants.*;
import frc.robot.Subsystems.*;

public class RobotContainer {

  XboxController controller = new XboxController(Constants.Controller.port);

  // Physical Components //
  public LegSubsystem leftLeg;
  public LegSubsystem rightLeg;
  public ArmSubsystem leftArm;
  public ArmSubsystem rightArm;

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
            false);

    leftArm.setPID(Arms.LeftPID.P, Arms.LeftPID.I, Arms.LeftPID.D);

    rightArm =
        new ArmSubsystem(
            DeviceIDs.rightArmMotor,
            Arms.Positions.rightMaxPosition,
            Arms.Positions.rightMinPosition,
            true);

    rightArm.setPID(Arms.RightPID.P, Arms.RightPID.I, Arms.RightPID.D);

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

    BooleanEvent toggleSafeMode =
        new BooleanEvent(loop, () -> controller.getRawButton(Controls.XboxController.enableButton));

    toggleSafeMode
        .rising()
        .ifHigh(
            () -> {
              rightLeg.setEnabled(!rightLeg.isEnabled());
              leftLeg.setEnabled(!leftLeg.isEnabled());
              rightArm.setEnabled(!rightArm.isEnabled());
              leftArm.setEnabled(!rightArm.isEnabled());
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
  }

  public void periodic() {
    loop.poll();
    leftLeg.run();
    rightLeg.run();
    leftArm.run();
    rightArm.run();
    updateShuffle();
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
}
