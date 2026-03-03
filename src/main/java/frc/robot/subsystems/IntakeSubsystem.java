package frc.robot.subsystems;

import com.revrobotics.spark.SparkAnalogSensor;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
public class IntakeSubsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */

  // Initilization
private final SparkMax m_IntakeArm = new SparkMax(IntakeConstants.kIntakeArmID, MotorType.kBrushless);
private final SparkMax m_IntakeWheel= new SparkMax(IntakeConstants.kIntakeWheelID, MotorType.kBrushless);
private final SparkClosedLoopController m_pidController1 = m_IntakeArm.getClosedLoopController();

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

  
/*
 * the runIntake and stopIntake methods are broken down
 * assuming the intaker will be kept out during games not frequently withdrawn and released
 * default state will be intakeIn(), and it will later be called again at the end of auto and teleop for climb
 */

  public void runIntake() {
    m_IntakeWheel.setVoltage(-3);
    }

  public void stopIntake() {
    m_IntakeWheel.setVoltage(0);
  }
  
  //intake out will be scheduled separately using on true logic
  public void intakeOut(double setposition)
  {
    m_pidController1.setSetpoint(setposition, com.revrobotics.spark.SparkBase.ControlType.kPosition);
  }
  
  //default state is set point 0, parameter omitted
  public void intakeIn()
  {
    m_pidController1.setSetpoint(0, com.revrobotics.spark.SparkBase.ControlType.kPosition);
  }

  

}