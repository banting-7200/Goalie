// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.*;
import frc.robot.Subsystems.LegSubsystem;

public class RobotContainer {

  static XboxController controller = new XboxController(Constants.Controller.port);
  private LegSubsystem leftLeg;

  public RobotContainer() {
    leftLeg = new LegSubsystem(DeviceIDs.leftLegMotor);
    configureBindings();
  }

  private void configureBindings() {
    // new JoystickButton(controller, Controller.Buttons.toggleLegs).debounce(3);
    new JoystickButton(controller, Controller.Buttons.toggleLegs)
        // .debounce(3)
        // .onTrue(new InstantCommand(() -> System.out.println("Clicked")));
        // .debounce(3);
        .onTrue(new InstantCommand(() -> leftLeg.togglePosition()));
    // .onTrue(new InstantCommand(() -> System.out.println("Debounced")));
  }
}
