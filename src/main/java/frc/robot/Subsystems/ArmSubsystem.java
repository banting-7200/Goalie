package frc.robot.Subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Arms;

public class ArmSubsystem extends SubsystemBase {

  private CANSparkMax motor;
  private AbsoluteEncoder encoder;
  private SparkPIDController PIDController;
  private double armAngle;
  private double upPositionEncoderValue;
  private double downPositionEncoderValue;
  private double currentPosition;
  private double setPoint;
  private boolean holdPosition = false;
  private int PIDSlot = 0;

  public ArmSubsystem(int deviceID, double upPositionEncoderValue, double downPositionEncoderValue, boolean isInverted) {
    this.upPositionEncoderValue = upPositionEncoderValue;
    this.downPositionEncoderValue = downPositionEncoderValue;
    setPoint = upPositionEncoderValue;

    motor = new CANSparkMax(deviceID, MotorType.kBrushless);
    PIDController = motor.getPIDController();
    encoder = motor.getAbsoluteEncoder();
    motor.setSmartCurrentLimit(Arms.motorControllerConfigurations.currentLimit);
    motor.setInverted(isInverted);
    motor.burnFlash();

    PIDController.setP(Arms.PID.P);
    PIDController.setI(Arms.PID.I);
    PIDController.setD(Arms.PID.D);
    PIDController.setFF(0);
    PIDController.setIZone(1.5, PIDSlot);
    PIDController.setFeedbackDevice(encoder);
    PIDController.setPositionPIDWrappingEnabled(false);
    PIDController.setOutputRange(-1, 1);

  }
  
  public void toggleHoldPosition() {
    holdPosition = !holdPosition;
  }

  public void run() {
    currentPosition = getEncoderPosition();
    if (holdPosition && withinSoftLimits()) {
      PIDController.setReference(setPoint, CANSparkMax.ControlType.kPosition, PIDSlot);
    } else {
      motor.set(0);
    }

  }
  
  private boolean withinSoftLimits() {
    currentPosition = getEncoderPosition();
    if (currentPosition < upPositionEncoderValue + Arms.Positions.stopRange
        && currentPosition > downPositionEncoderValue - Arms.Positions.stopRange) {
      System.out.println(currentPosition);
      return true;
    }
    System.out.println("SOFT LIMITS: " + (downPositionEncoderValue - Arms.Positions.stopRange) + " | " + currentPosition
        + " | " + upPositionEncoderValue + Arms.Positions.stopRange);
    return false;
  }

  public void moveFromRange(double position) {
    if (position > 1) 
      position = 1;
    if (position < -1) 
      position = -1;
      
    position = (position + 1) / 2 * (upPositionEncoderValue - downPositionEncoderValue) + downPositionEncoderValue;
       
    moveToAngle(position);
    System.out.println(position);
  }

  public void moveToAngle(double armAngle) {
    this.armAngle = armAngle;
  }

  private double getEncoderPosition() {
    return encoder.getPosition();
  }
}
