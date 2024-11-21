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
  private double upPosition = Legs.Positions.upPosition;
  private double downPosition = Legs.Positions.downPosition;

  private int PIDSlot = 0;

  public LegSubsystem(int deviceID) {
    motor = new CANSparkMax(deviceID, MotorType.kBrushless);
    encoder = motor.getAbsoluteEncoder();

    PIDController = motor.getPIDController();
    PIDController.setP(Legs.PID.P, PIDSlot);
    PIDController.setI(Legs.PID.I, PIDSlot);
    PIDController.setD(Legs.PID.D, PIDSlot);
    PIDController.setPositionPIDWrappingEnabled(false);
    PIDController.setOutputRange(-1, 1);
    PIDController.setFeedbackDevice(encoder);
  }

  public void togglePosition() {
    setPosition(getEncoderPosition() > upPosition + stopRange);
  }

  public void setPosition(boolean up) {
    if (up) {
      setMotorPosition(upPosition);
    } else {
      setMotorPosition(downPosition);
    }
  }

  private void setMotorPosition(double setpoint) {
    double currentPosition = encoder.getPosition();
    while (currentPosition < setpoint + stopRange && currentPosition > setpoint - stopRange) {
      PIDController.setReference(setpoint, CANSparkMax.ControlType.kPosition, PIDSlot);
      currentPosition = encoder.getPosition();
    }
  }

  private void setMotorSpeed(double speed) {
    motor.set(speed);
  }

  private double getEncoderPosition() {
    return encoder.getPosition();
  }
}
