package frc.robot.subsystems;

import com.revrobotics.spark.SparkAnalogSensor;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.intakeConstants;;
public class IntakeSubsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */

  // Initilization
private final SparkMax m_IntakeArm = new SparkMax(intakeConstants.kIntakeArmID, MotorType.kBrushless);
private final SparkMax m_IntakeWheel= new SparkMax(intakeConstants.kIntakeWheelID, MotorType.kBrushless);
private final SparkClosedLoopController m_pidController = m_IntakeArm.getClosedLoopController();
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
    SmartDashboard.putNumber("Kicker Encoder Position", m_ArmEncoder.getPosition());
    SmartDashboard.putNumber("Kicker Set Point Value",intakeConstants.kIntakeSetpoint);
  }

  

  public void runIntake(double setposition) {
    m_IntakeWheel.setVoltage(4);
    m_pidController.setSetpoint(setposition, com.revrobotics.spark.SparkBase.ControlType.kPosition);

  }

  public void stopIntake(double setposition) {
    m_IntakeWheel.setVoltage(0);
    //m_pidController.setReference(0.1, com.revrobotics.spark.SparkBase.ControlType.kPosition);
    m_pidController.setSetpoint(setposition, com.revrobotics.spark.SparkBase.ControlType.kPosition);

  }

}