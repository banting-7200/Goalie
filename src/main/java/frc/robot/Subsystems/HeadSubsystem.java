package frc.robot.Subsystems;

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
  boolean doesCodeHaveMotorPriority = true;

  public HeadSubsystem(int headMotorID, int lowerLimitSwitchID, int upperLimitSwitchID) {
    headMotor = new TalonFX(headMotorID);
    lowerLimitSwitch = new DigitalInput(lowerLimitSwitchID);
    upperLimitSwitch = new DigitalInput(upperLimitSwitchID);
    headMotor.configFactoryDefault();
    headMotor.configSelectedFeedbackSensor(
        TalonFXFeedbackDevice.IntegratedSensor, PIDControllerSlot, timeOutMs);
    headMotor.setSensorPhase(true);
    headMotor.setInverted(true);
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
    if (lowerLimitSwitch.get() && upperLimitSwitch.get()) {
      return true;
    }
    // System.out.println("Limits hit: " + lowerLimitSwitch.get() + " | " + upperLimitSwitch.get());
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

  public void setHeadPosition(double rangeMin, double rangeMax, double input) {
    double position =
        (input - rangeMin)
                / (rangeMax - rangeMin)
                * (Head.Positions.maxPosition - Head.Positions.minPosition)
            + Head.Positions.minPosition;
    setPoint = position;
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
    headMotor.set(ControlMode.PercentOutput, 0);
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
    } else if (!doesCodeHaveMotorPriority) {
      // headMotor.set(ControlMode.PercentOutput, 0);
    }
  }
}
