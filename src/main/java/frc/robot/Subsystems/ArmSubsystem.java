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

  private double currentPosition;
  private double setPosition;
  private double upPosition;
  private double downPosition;

  private double upperStopRange = Arms.Positions.upperStopRange;
  private double lowerStopRange = Arms.Positions.lowerStopRange;

  private boolean enabled = false;

  public ArmSubsystem(int deviceID, double upPosition, double downPosition, boolean isInverted) {

    this.upPosition = upPosition;
    this.downPosition = downPosition;
    setPosition = downPosition;
    setPosition = downPosition;

    motor = new CANSparkMax(deviceID, MotorType.kBrushless);
    encoder = motor.getAbsoluteEncoder();
    PIDController = motor.getPIDController();

    motor.setSmartCurrentLimit(Arms.motorControllerConfigurations.currentLimit);
    motor.setInverted(isInverted);
    motor.burnFlash();

    PIDController.setP(0);
    PIDController.setI(0);
    PIDController.setD(0);
    PIDController.setFF(0);
    PIDController.setIZone(1.5);
    PIDController.setFeedbackDevice(encoder);
    PIDController.setPositionPIDWrappingEnabled(false);
    PIDController.setOutputRange(-1, 1);
  }

  public void setEnabled(boolean enabled) {
    this.enabled = enabled;
  }

  public boolean isEnabled() {
    return enabled;
  }

  public void run() {
    currentPosition = encoder.getPosition();
    if (enabled) {
      if (setPosition > currentPosition) {
        if (!withinUpperLimits()) return;
      } else {
        if (!withinLowerLimits()) return;
      }
      PIDController.setReference(setPosition, CANSparkMax.ControlType.kPosition);
    } else {
      motor.set(0);
    }
  }

  private boolean withinUpperLimits() {
    return (currentPosition < upPosition - upperStopRange);
  }

  private boolean withinLowerLimits() {
    return (currentPosition > downPosition - lowerStopRange);
  }

  /**
   * Moves the arm to a position based on where an input is within a given range.
   *
   * @param rangeMax the maximum value of the range
   * @param rangeMin the minimum value of the range
   * @param input the value within this range to map the arm position to
   */
  public void moveFromRange(double rangeMin, double rangeMax, double input) {
    if (input > rangeMax) input = rangeMax;
    if (input < rangeMin) input = rangeMin;

    double position =
        (input - rangeMin) / (rangeMax - rangeMin) * (upPosition - downPosition) + downPosition;

    moveToAngle(position);
    System.out.println(position);
  }

  public void moveToAngle(double setPosition) {
    this.setPosition = setPosition;
  }

  public double getPosition() {
    return currentPosition;
  }

  public void setPID(double P, double I, double D) {
    PIDController.setP(P);
    PIDController.setI(I);
    PIDController.setD(D);
  }

  public void setPID(double[] PID) {
    PIDController.setP(PID[0]);
    PIDController.setI(PID[1]);
    PIDController.setD(PID[2]);
  }

  public double[] getPID() {
    return new double[] {PIDController.getP(), PIDController.getI(), PIDController.getD()};
  }

  public double getCurrent() {
    return motor.getOutputCurrent();
  }
}
