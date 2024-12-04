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

  private double upPositionEncoderValue;
  private double downPositionEncoderValue;
  private double currentPosition;
  private double setPoint;

  private boolean legPositionUp = true;
  private boolean holdPosition = false;

  private int PIDSlot = 0;

  public LegSubsystem(
      int deviceID,
      double downPositionEncoderValue,
      double upPositionEncoderValue,
      boolean isInverted) {

    this.downPositionEncoderValue = downPositionEncoderValue;
    this.upPositionEncoderValue = upPositionEncoderValue;
    setPoint = upPositionEncoderValue;

    motor = new CANSparkMax(deviceID, MotorType.kBrushless);
    encoder = motor.getAbsoluteEncoder();
    PIDController = motor.getPIDController();

    PIDController.setP(Legs.PID.P);
    PIDController.setI(Legs.PID.I);
    PIDController.setD(Legs.PID.D);
    PIDController.setFF(0);
    PIDController.setIZone(1.5, PIDSlot);
    PIDController.setOutputRange(-1, 1);
    PIDController.setFeedbackDevice(encoder);
    PIDController.setPositionPIDWrappingEnabled(false);

    // motor.restoreFactoryDefaults();
    motor.setSmartCurrentLimit(Legs.motorControllerConfigurations.currentLimit);
    motor.setIdleMode(IdleMode.kBrake);
    motor.setInverted(isInverted);
    motor.burnFlash();
  }

  public void togglePosition() {
    legPositionUp = !legPositionUp;
    if (legPositionUp) {
      setPositionSetPoint(upPositionEncoderValue);
    } else {
      setPositionSetPoint(downPositionEncoderValue);
    }
    // setPositionSetPoint(90);

  }

  public void toggleHoldPosition() {
    this.holdPosition = !this.holdPosition;
  }

  public void setHoldPosition(boolean holdPosition) {
    this.holdPosition = holdPosition;
  }

  public void setPositionSetPoint(double setPoint) {
    this.setPoint = setPoint;
  }

  public double getCurrentSetPoint() {
    return setPoint;
  }

  public void run() {
    currentPosition = encoder.getPosition();
    if (legPositionUp && holdPosition) {
      if (withinUpperSoftLimits()) {
        PIDController.setReference(setPoint, CANSparkMax.ControlType.kPosition, PIDSlot);
      } else {
        motor.set(0);
        // System.out.println("upper soft");
      }
    } else if (!legPositionUp && holdPosition) {
      if (withinLowerSoftLimits()) {
        PIDController.setReference(setPoint, CANSparkMax.ControlType.kPosition, PIDSlot);
      } else {
        motor.set(0);
        // System.out.println("lower soft");
      }
    } else {
      motor.set(0);
    }
  }

  private boolean withinLowerSoftLimits() {
    // System.out.println(
    //     "LOWER SOFT LIMITS "
    //         + downPositionEncoderValue
    //         + " || "
    //         + currentPosition
    //         + " || "
    //         + upPositionEncoderValue);
    return (currentPosition > downPositionEncoderValue + lowerStopRange);
  }

  private boolean withinUpperSoftLimits() {
    // System.out.println(
    //     "UPPER SOFT LIMITS "
    //         + downPositionEncoderValue
    //         + " || "
    //         + currentPosition
    //         + " || "
    //         + upPositionEncoderValue);
    return (currentPosition < upPositionEncoderValue - upperStopRange);
  }

  public boolean isUp() {
    return legPositionUp;
  }

  public boolean isLocked() {
    return holdPosition;
  }

  public void setPID(double[] PID) {
    PIDController.setP(PID[0]);
    PIDController.setI(PID[1]);
    PIDController.setD(PID[2]);
  }
}
