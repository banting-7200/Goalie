package frc.robot.Subsystems;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants.Head;

public class HeadSubsystem {

  TalonFX headMotor;
  DigitalInput upperLimitSwitch;
  DigitalInput lowerLimitSwitch;
  DutyCycleOut dutyCycleMotorRequest = new DutyCycleOut(0.0);
  PositionVoltage positionMotorRequest = new PositionVoltage(0).withSlot(0);
  double setPoint;
  double currentPosition;
  int timeOutMs = 30;
  int PIDControllerSlot = 0;
  boolean upPosition = false;
  boolean enabledMovement = false;
  boolean doesCodeHaveMotorPriority = false;
  long currentMillis = System.currentTimeMillis(), previousTime = System.currentTimeMillis();

  public HeadSubsystem(int headMotorID, int lowerLimitSwitchID, int upperLimitSwitchID) {
    headMotor = new TalonFX(headMotorID, "rio");
    lowerLimitSwitch = new DigitalInput(lowerLimitSwitchID);
    upperLimitSwitch = new DigitalInput(upperLimitSwitchID);

    TalonFXConfiguration configs = new TalonFXConfiguration();
    var slot0Configs = new Slot0Configs();
    configs.Slot0.kP = Head.PID.P;
    configs.Slot0.kI = Head.PID.I;
    configs.Slot0.kD = Head.PID.D;
    slot0Configs.kP = Head.PID.P;
    slot0Configs.kI = Head.PID.I;
    slot0Configs.kD = Head.PID.D;

   // headMotor.getConfigurator().apply(configs);
    headMotor.getConfigurator().apply(slot0Configs);



  }

  public void enableMovement(boolean enabledMovement) {
    this.enabledMovement = enabledMovement;
  }

  public boolean isEnabled() {
    return enabledMovement;
  }

  public boolean withinLimits() {
    if (!lowerLimitSwitch.get() && upperLimitSwitch.get()) {
      return true;
    }
    System.out.println("Limits hit: " + lowerLimitSwitch.get() + " | " + !upperLimitSwitch.get());
    return false;
  }

  public void toggleHead() {
    upPosition = !upPosition;
    if (upPosition) {
      setPoint = Head.Positions.maxPosition;
    } else {
      setPoint = Head.Positions.minPosition;
    }
  }

  public double getCurrentPosition() {

    // currentPosition = headMotor.getSelectedSensorPosition();
    // return headMotor.getSelectedSensorPosition();
    currentPosition = headMotor.getPosition().getValue();
    return currentPosition;
  }

  public void zeroEncoder() {
    doesCodeHaveMotorPriority = true;
    while (withinLimits() && enabledMovement) {
      headMotor.setControl(dutyCycleMotorRequest.withOutput(0.1));
      System.out.println("moving");
    }
    System.out.println("hit zero limit");
    headMotor.setPosition(0);
    setPoint = Head.Positions.minPosition;
    positionMotorRequest.Position = setPoint;
    upPosition = false;
    headMotor.setControl(positionMotorRequest);
    doesCodeHaveMotorPriority = false;
  }

  public void testReZeroEncoder() {
    headMotor.setPosition(0);
  }

  public void run() {
    positionMotorRequest.Position = setPoint;
    if (withinLimits() && enabledMovement && !doesCodeHaveMotorPriority) {
      headMotor.setControl(positionMotorRequest);
    } else {
      headMotor.setControl(dutyCycleMotorRequest.withOutput(0));
    }
  }

  public void testRun() {
    dutyCycleMotorRequest.Output = 0.2;

     headMotor.setControl(dutyCycleMotorRequest);

  }
}
