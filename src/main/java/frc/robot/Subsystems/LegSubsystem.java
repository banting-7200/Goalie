package frc.robot.Subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Legs;

public class LegSubsystem extends SubsystemBase {

  private CANSparkMax motor;
  private AbsoluteEncoder encoder;
  private SparkPIDController PIDController;

  private double upperStopRange = Legs.Positions.upperStopRange;
  private double lowerStopRange = Legs.Positions.lowerStopRange;

  private double upPosition;
  private double downPosition;
  private double currentPosition;
  private double setPosition;

  private boolean isUp = true;
  private boolean enabled = false;

  public LegSubsystem(int deviceID, double downPosition, double upPosition, boolean isInverted) {

    this.downPosition = downPosition;
    this.upPosition = upPosition;
    setPosition = upPosition;

    motor = new CANSparkMax(deviceID, MotorType.kBrushless);
    encoder = motor.getAbsoluteEncoder();
    PIDController = motor.getPIDController();

    PIDController.setP(Legs.PID.P);
    PIDController.setI(Legs.PID.I);
    PIDController.setD(Legs.PID.D);
    PIDController.setFF(0);
    PIDController.setIZone(1.5);
    PIDController.setOutputRange(-1, 1);
    PIDController.setFeedbackDevice(encoder);
    PIDController.setPositionPIDWrappingEnabled(false);

    motor.setSmartCurrentLimit(Legs.motorControllerConfigurations.currentLimit);
    motor.setIdleMode(IdleMode.kBrake);
    motor.setInverted(isInverted);
    motor.burnFlash();
  }

  public void togglePosition() {
    isUp = !isUp;
    if (isUp) {
      setPositionsetPosition(upPosition);
    } else {
      setPositionsetPosition(downPosition);
    }
  }

  public void setEnabled(boolean enabled) {
    this.enabled = enabled;
  }

  public void setPositionsetPosition(double setPosition) {
    this.setPosition = setPosition;
  }

  public double getCurrentsetPosition() {
    return setPosition;
  }

  public void run() {
    currentPosition = encoder.getPosition();
    if (isUp && enabled) {
      if (withinUpperSoftLimits()) {
        PIDController.setReference(setPosition, CANSparkMax.ControlType.kPosition);
      } else {
        motor.set(0);
      }
    } else if (!isUp && enabled) {
      if (withinLowerSoftLimits()) {
        PIDController.setReference(setPosition, CANSparkMax.ControlType.kPosition);
      } else {
        motor.set(0);
      }
    } else {
      motor.set(0);
    }
  }

  private boolean withinLowerSoftLimits() {
    // System.out.println(
    //     "LOWER SOFT LIMITS "
    //         + downPosition
    //         + " || "
    //         + currentPosition
    //         + " || "
    //         + upPosition);
    return (currentPosition > downPosition + lowerStopRange);
  }

  private boolean withinUpperSoftLimits() {
    // System.out.println(
    //     "UPPER SOFT LIMITS "
    //         + downPosition
    //         + " || "
    //         + currentPosition
    //         + " || "
    //         + upPosition);
    return (currentPosition < upPosition - upperStopRange);
  }

  public boolean isUp() {
    return isUp;
  }

  public boolean isEnabled() {
    return enabled;
  }

  public void setPID(double[] PID) {
    PIDController.setP(PID[0]);
    PIDController.setI(PID[1]);
    PIDController.setD(PID[2]);
  }
}
