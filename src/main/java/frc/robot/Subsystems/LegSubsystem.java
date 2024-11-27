package frc.robot.Subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Legs;

public class LegSubsystem extends SubsystemBase {
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

  public LegSubsystem(int deviceID) {
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
  }

  public void togglePosition() {
    currentPosition = getEncoderPosition();
    legPositionToggle = !legPositionToggle;
    if (legPositionToggle) {
      setPositionSetPoint(upPositionEncoderValue);
    } else {
      setPositionSetPoint(downPositionEncoderValue);
    }
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

    while (!checkErrorRange() && !checkSoftLimits()) {
      System.out.println("Trying to turn");
      PIDController.setReference(setPoint, CANSparkMax.ControlType.kPosition, PIDSlot);
      currentPosition = encoder.getPosition();
    }
  }

  private boolean checkSoftLimits() {
    if (currentPosition > upPositionEncoderValue
        || currentPosition
            < downPositionEncoderValue) { // soft limits, wont move if not inside up and down
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
