// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.event.BooleanEvent;
import edu.wpi.first.wpilibj.event.EventLoop;
import frc.robot.Constants.*;
import frc.robot.Subsystems.LegSubsystem;
import frc.robot.Subsystems.ShuffleboardSubsystem;

public class RobotContainer {

  static XboxController controller = new XboxController(Constants.Controller.port);

  // Physical Components //
  public LegSubsystem leftLeg;
  public LegSubsystem rightLeg;

  private EventLoop loop = new EventLoop();
  private ShuffleboardSubsystem shuffle = ShuffleboardSubsystem.getInstance();

  public RobotContainer() {
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
    configureBindings();
  }

  private void configureBindings() {
    shuffle.setPID("PID Tuner", 1, 0, 0);

    BooleanEvent toggleLeftLeg = new BooleanEvent(loop, () -> controller.getXButton());

    toggleLeftLeg.rising().ifHigh(() -> leftLeg.togglePosition());

    BooleanEvent toggleRightLeg = new BooleanEvent(loop, () -> controller.getBButton());

    toggleRightLeg.rising().ifHigh(() -> rightLeg.togglePosition());

    BooleanEvent toggleSafeMode = new BooleanEvent(loop, () -> controller.getYButton());

    toggleSafeMode
        .rising()
        .ifHigh(
            () -> {
              rightLeg.toggleHoldPosition();
              leftLeg.toggleHoldPosition();
            });

    BooleanEvent updatePIDs = new BooleanEvent(loop, () -> controller.getAButton());

    updatePIDs
        .rising()
        .ifHigh(
            () -> {
              double[] PID = shuffle.getPID("PID Tuner");
              //Simply change the below line to tune PIDs for another object.
              leftLeg.setPID(PID);
              System.out.println("UPDATING PIDS");
            });
  }

  public void pollLoop() {
    loop.poll();
    leftLeg.run();
    rightLeg.run();
    updateShuffle();
  }

  public void updateShuffle() {
    shuffle.setTab("Status");

    shuffle.setBoolean("Left Leg Up", leftLeg.isUp());
    shuffle.setBoolean("Left Leg Locked", leftLeg.isLocked());

    shuffle.setBoolean("Right Leg Up", rightLeg.isUp());
    shuffle.setBoolean("Right Leg Locked", rightLeg.isLocked());
  }
}
