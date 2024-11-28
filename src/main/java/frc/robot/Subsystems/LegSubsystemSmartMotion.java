package frc.robot.Subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Legs;

public class LegSubsystemSmartMotion extends SubsystemBase {
  private CANSparkMax motor;
  private AbsoluteEncoder encoder;
  private SparkPIDController PIDController;

  private double stopRange = Legs.Positions.stopRange;
  private double upPositionEncoderValue = Legs.Positions.upPosition;
  private double downPositionEncoderValue = Legs.Positions.downPosition;
  private double currentPosition;
  private double setPoint;
  private boolean legPositionToggle = true;

  private int PIDSlot = 0;

  public LegSubsystemSmartMotion(int deviceID) {
    motor = new CANSparkMax(deviceID, MotorType.kBrushless);
    encoder = motor.getAbsoluteEncoder();

    PIDController = motor.getPIDController();
    PIDController.setP(Legs.PID.P);
    PIDController.setI(Legs.PID.I);
    PIDController.setD(Legs.PID.D);
    PIDController.setFF(
        0); // 0 is default, only used in velocity control type, in which, its different for each
    // motor
    PIDController.setIZone(1.5, PIDSlot); // if I goes past this, stop using I
    PIDController.setOutputRange(-1, 1);
    PIDController.setFeedbackDevice(encoder);
    PIDController.setSmartMotionMinOutputVelocity(Legs.motorSpeeds.fastMinVel, PIDSlot);
    PIDController.setSmartMotionMaxVelocity(Legs.motorSpeeds.fastMaxVel, PIDSlot);
    PIDController.setSmartMotionMaxAccel(Legs.motorSpeeds.motorAccel, PIDSlot);
    PIDController.setSmartMotionAllowedClosedLoopError(Legs.motorSpeeds.allowedError, PIDSlot);
  }

  public void togglePosition() {
    currentPosition = getEncoderPosition();
    System.out.println(currentPosition);
    legPositionToggle = !legPositionToggle;
    // if (legPositionToggle) {
    //   setPositionSetPoint(upPositionEncoderValue);
    // } else {
    //   setPositionSetPoint(downPositionEncoderValue);
    // }
    setPositionSetPoint(1);
    moveMotorToPosition();
  }

  private boolean checkErrorRange() {
    currentPosition = getEncoderPosition();
    return Math.abs(currentPosition - setPoint) < stopRange;
  }

  public void setPositionSetPoint(double setPoint) {
    this.setPoint = setPoint;
  }

  public double getCurrentSetPoint() {
    return setPoint;
  }

  private void moveMotorToPosition() {
    currentPosition = encoder.getPosition();
    System.out.println(
        "trying to move, limits stopped movement " + Math.abs(currentPosition - setPoint));

    while (!checkSoftLimits()) {
      System.out.println("Trying to turn");
      PIDController.setReference(setPoint, CANSparkMax.ControlType.kPosition, PIDSlot);
      currentPosition = encoder.getPosition();
    }
  }

  private boolean checkSoftLimits() {
    if (currentPosition > (upPositionEncoderValue + stopRange)
        || currentPosition
            < (downPositionEncoderValue
                - stopRange)) { // soft limits, wont move if not inside up and down
      // positions
      System.out.println(
          "SOFT LIMITS "
              + downPositionEncoderValue
              + " || "
              + currentPosition
              + " || "
              + upPositionEncoderValue);
      return true;
    } else {

      return false;
    }
  }

  private double getEncoderPosition() {
    return encoder.getPosition();
  }
}
