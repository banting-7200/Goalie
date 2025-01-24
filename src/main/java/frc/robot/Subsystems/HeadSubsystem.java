package frc.robot.Subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants.Head;

public class HeadSubsystem {

  TalonFX headMotor;
  DigitalInput upperLimitSwitch;
  DigitalInput lowerLimitSwitch;
  double setPoint;
  double currentPosition;
  boolean upPosition = false;
  boolean enabledMovement = false;
  boolean readyToMove = false;
  boolean doesCodeHaveMotorPriority = false;
  long currentMillis = System.currentTimeMillis(), previousTime = System.currentTimeMillis();

  public HeadSubsystem(int headMotorID, int lowerLimitSwitchID, int upperLimitSwitchID) {
    headMotor = new TalonFX(headMotorID);
    lowerLimitSwitch = new DigitalInput(lowerLimitSwitchID);
    upperLimitSwitch = new DigitalInput(upperLimitSwitchID);
  }

  public void enableMovement(boolean enableMovement) {
    this.enabledMovement = enabledMovement;
  }

  public boolean withinLimits() {
    if (!lowerLimitSwitch.get() && !upperLimitSwitch.get()) {
      return true;
    }
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

  public void readEncoder() {
    currentPosition = headMotor.getSelectedSensorPosition();
  }

  public void zeroEncoder() {
    doesCodeHaveMotorPriority = true;
    while (lowerLimitSwitch.get() == false && enabledMovement) {
      headMotor.set(ControlMode.PercentOutput, -0.05);
    }
    headMotor.setSelectedSensorPosition(0);
    headMotor.set(ControlMode.Position, Head.Positions.minPosition);
    upPosition = false;
    if (headMotor.getClosedLoopError() <= 10) {
      doesCodeHaveMotorPriority = false;
    }
  }

  public void run() {

    if (withinLimits() && enabledMovement && !doesCodeHaveMotorPriority) {
      headMotor.set(ControlMode.Position, setPoint);
    } else {
      headMotor.set(ControlMode.PercentOutput, 0);
    }
  }
}
