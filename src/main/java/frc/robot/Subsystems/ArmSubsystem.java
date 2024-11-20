package frc.robot.Subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkLimitSwitch;
import com.revrobotics.SparkPIDController;
import frc.robot.Constants;

public class ArmSubsystem extends SubsystemBase {
   
  private CANSparkMax motor;
  private AbsoluteEncoder legEncoder;
  private SparkPIDController pidController;
  private double armAngle;
  private double downEncoderHardStopPosition = 0;
  private double upEncoderHardStopPosition = 50;
  private double


  public ArmSubsystem(int deviceID, double initalArmAngle) {
    motor = new CANSparkMax(deviceID, MotorType.kBrushless);
    pidController = motor.getPIDController();
    pidController.setP(0.005);
    pidController.setI(0);
    pidController.setD(0.002);
    pidController.setPositionPIDWrappingEnabled(false);
    pidController.setOutputRange(-1, 1);
    this.armAngle = armAngle;
  }

  public void moveToAngle(double armAngle) {
    this.armAngle = armAngle;
  }

  public void setMotorSpeed() {

  }
  
  public void getEncoderPosition() {

  }

  public void moveToReadyPosition() {

  }
  
  public void moveToResetPosition() {

  }
  
  public void checkEncoderStop() {
    
  }
  


  


}