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
  private static ShuffleboardSubsystem shuffle;

  private double stopRange = Legs.Positions.stopRange;
  private double upPositionEncoderValue = Legs.Positions.upPosition;
  private double downPositionEncoderValue = Legs.Positions.downPosition;
  private double currentPosition;
  private double setPoint;
  private boolean legPositionUp = true;
  private boolean holdPosition = false;
  private int deviceID;

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

    shuffle.setPID(
        "Motor: " + deviceID, PIDController.getP(), PIDController.getI(), PIDController.getD());
  }

  public void togglePosition() {
    System.out.println(currentPosition);
    legPositionUp = !legPositionUp;
    if (legPositionUp) {
      setPositionSetPoint(upPositionEncoderValue - 5);
    } else {
      setPositionSetPoint(downPositionEncoderValue + 5);
    }
    // setPositionSetPoint(90);

  }

  public void toggleHoldPosition() {
    this.holdPosition = !this.holdPosition;
  }

  public void setPositionSetPoint(double setPoint) {
    this.setPoint = setPoint;
  }

  public double getCurrentSetPoint() {
    return setPoint;
  }

  public void run() {
    currentPosition = encoder.getPosition();
    if (withinSoftLimits() && this.holdPosition) {
      System.out.println("Trying to turn || " + currentPosition + " setpoint: " + setPoint);
      PIDController.setReference(setPoint, CANSparkMax.ControlType.kPosition, PIDSlot);
    }
  }

  private boolean withinSoftLimits() {
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
      return false;
    } else {

      return true;
    }
  }

  private double getEncoderPosition() {
    return encoder.getPosition();
  }

  public void updateShuffe() {
    double[] PID = shuffle.getPID("Motor: " + deviceID);
    PIDController.setP(PID[0]);
    PIDController.setI(PID[1]);
    PIDController.setD(PID[2]);
  }
}
