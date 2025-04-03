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
  boolean enabledMovement = true;
  boolean isZeroed = false;

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
    return (lowerLimitSwitch.get() && upperLimitSwitch.get());
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

  public void reZero() {
    isZeroed = false;
  }

  public void zeroEncoderPeriodic() {
    if (isZeroed) return;
    if (!enabledMovement) return;
    if (lowerLimitSwitch.get()) {
      headMotor.set(ControlMode.PercentOutput, -0.06);
    } else {
      headMotor.set(ControlMode.PercentOutput, 0);
      System.out.println("Head Zeroed");
      headMotor.setSelectedSensorPosition(0);
      setPoint = Head.Positions.minPosition;
      upPosition = false;
      isZeroed = true;
      headMotor.set(ControlMode.Position, setPoint);
    }
  }

  public void run() {
    zeroEncoderPeriodic();
    if (withinLimits() && enabledMovement) {
      headMotor.set(ControlMode.Position, setPoint);
    }
  }
}
