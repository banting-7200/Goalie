// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.event.BooleanEvent;
import edu.wpi.first.wpilibj.event.EventLoop;
import frc.robot.Constants.*;
import frc.robot.Subsystems.LegSubsystem;

public class RobotContainer {

  static XboxController controller = new XboxController(Constants.Controller.port);
  public LegSubsystem leftLeg;
  public LegSubsystem rightLeg;
  private EventLoop loop = new EventLoop();

  public RobotContainer() {
    leftLeg =
        new LegSubsystem(
            "Left Leg",
            DeviceIDs.leftLegMotor,
            Legs.Positions.leftDownPosition,
            Legs.Positions.leftUpPosition);
    rightLeg =
        new LegSubsystem(
            "Right Leg",
            DeviceIDs.rightLegMotor,
            Legs.Positions.rightDownPosition,
            Legs.Positions.rightUpPosition);
    configureBindings();
  }

  private void configureBindings() {

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
              rightLeg.updateShuffe();
              leftLeg.updateShuffe();
              System.out.println("UPDATING PIDS");
            });
    // new JoystickButton(controller, Controller.Buttons.toggleLegs).debounce(3);
    // new JoystickButton(controller, Controller.Buttons.toggleLegs)
    // .debounce(3)
    // .onTrue(new InstantCommand(() -> System.out.println("Clicked")));
    // .debounce(3);
    // .onTrue(new InstantCommand(() -> leftLeg.togglePosition()));
    // .onTrue(new InstantCommand(() -> System.out.println("Debounced")));
  }

  public void pollLoop() {
    loop.poll();
    leftLeg.run();
    rightLeg.run();
  }
}
