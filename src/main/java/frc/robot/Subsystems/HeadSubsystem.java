package frc.robot.Subsystems;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants.Head;

public class HeadSubsystem {

  TalonFX headMotor;
  DigitalInput upperLimitSwitch;
  DigitalInput lowerLimitSwitch;

  double setPoint;
  double currentPosition;
  int timeOutMs = 30;
  int PIDControllerSlot = 0;
  boolean upPosition = false;
  boolean enabledMovement = false;
  boolean doesCodeHaveMotorPriority = false;

  public HeadSubsystem(int headMotorID, int lowerLimitSwitchID, int upperLimitSwitchID) {
    headMotor = new TalonFX(headMotorID);
    lowerLimitSwitch = new DigitalInput(lowerLimitSwitchID);
    upperLimitSwitch = new DigitalInput(upperLimitSwitchID);
    headMotor.configFactoryDefault();
    headMotor.configSelectedFeedbackSensor(
        TalonFXFeedbackDevice.IntegratedSensor, PIDControllerSlot, timeOutMs);
    headMotor.setSensorPhase(true);
    headMotor.setInverted(false);
    headMotor.configPeakOutputForward(1, timeOutMs);
    headMotor.configPeakOutputReverse(-1, timeOutMs);
    headMotor.configNominalOutputForward(0, timeOutMs);
    headMotor.configNominalOutputReverse(0, timeOutMs);
    headMotor.configAllowableClosedloopError(0, 0, timeOutMs);
    headMotor.config_kP(PIDControllerSlot, Head.PID.P, timeOutMs);
    headMotor.config_kI(PIDControllerSlot, Head.PID.I, timeOutMs);
    headMotor.config_kD(PIDControllerSlot, Head.PID.D, timeOutMs);
  }

  public void enableMovement(boolean enabledMovement) {
    this.enabledMovement = enabledMovement;
  }

  public boolean isEnabled() {
    return enabledMovement;
  }

  public boolean withinLimits() {
    if (!lowerLimitSwitch.get()) {
      return true;
    }
    System.out.println("Limits hit: " + lowerLimitSwitch.get());
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
    currentPosition = headMotor.getSelectedSensorPosition();
    return headMotor.getSelectedSensorPosition();
  }

  public void zeroEncoder() {
    doesCodeHaveMotorPriority = true;
    while (withinLimits() && enabledMovement) {
      headMotor.set(ControlMode.PercentOutput, -0.06);
    }
    System.out.println("hit zero limit");
    headMotor.setSelectedSensorPosition(0);
    setPoint = Head.Positions.minPosition;
    upPosition = false;
    headMotor.set(ControlMode.Position, setPoint);
    doesCodeHaveMotorPriority = false;
  }

  public void testReZeroEncoder() {
    headMotor.setSelectedSensorPosition(0);
  }

  public void run() {
    if (withinLimits() && enabledMovement && !doesCodeHaveMotorPriority) {
      headMotor.set(ControlMode.Position, setPoint);
    } else {
      // headMotor.set(ControlMode.PercentOutput, 0);
    }
  }
}
