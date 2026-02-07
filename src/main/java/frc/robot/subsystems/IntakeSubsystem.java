package frc.robot.subsystems;

import com.revrobotics.spark.SparkAnalogSensor;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;

import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.intakeConstants;;
public class IntakeSubsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */

  // Initilization
private final SparkMax m_IntakeArm = new SparkMax(intakeConstants.kIntakeArmID, MotorType.kBrushless);
private final SparkMax m_IntakeWheel= new SparkMax(intakeConstants.kIntakeWheelID, MotorType.kBrushless);
private final SparkClosedLoopController m_pidController1 = m_IntakeArm.getClosedLoopController();
private final SparkClosedLoopController m_pidController2 = m_IntakeWheel.getClosedLoopController();

private SparkAnalogSensor m_ArmEncoder;
  public IntakeSubsystem() {

  
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    m_ArmEncoder = m_IntakeArm.getAnalog();
  }

  

  public void runIntake(double setposition) {
    // m_pidController2.setSetpoint(100, com.revrobotics.spark.SparkBase.ControlType.kVelocity);
    m_IntakeWheel.setVoltage(4);
    m_pidController1.setSetpoint(setposition, com.revrobotics.spark.SparkBase.ControlType.kPosition);
    System.out.println("willy out!");

  }

  public void stopIntake(double setposition) {
    // m_pidController2.setSetpoint(0, com.revrobotics.spark.SparkBase.ControlType.kVelocity);
    //m_pidController.setReference(0.1, com.revrobotics.spark.SparkBase.ControlType.kPosition);
    m_IntakeWheel.setVoltage(setposition);
    m_pidController1.setSetpoint(setposition, com.revrobotics.spark.SparkBase.ControlType.kPosition);
    System.out.println("willy in!");

  }

}