package frc.robot.subsystems;


import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
public class IntakeSubsystem extends SubsystemBase {
      public String m_intakeOn = "Off";

  /** Creates a new ExampleSubsystem. */

  // Initilization
private final SparkMax m_IntakeLeft = new SparkMax(IntakeConstants.kIntakeArmID, MotorType.kBrushless);
private final SparkMax m_IntakeRight= new SparkMax(IntakeConstants.kIntakeWheelID, MotorType.kBrushless);

public IntakeSubsystem() {
 
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   * 
   * @author Larry9297
   */

  @Override
  public void periodic() {
    SmartDashboard.putString("intakeon?", m_intakeOn);
    // This method will be called once per scheduler run
  }

  
/*
 * the runIntake and stopIntake methods are broken down
 * assuming the intaker will be kept out during games not frequently withdrawn and released
 * default state will be intakeIn(), and it will later be called again at the end of auto and teleop for climb
 */

  /**
   * Method used to run intake
   * 
   * @author Larry9297
   */
  public void runIntake() {
    m_IntakeLeft.setVoltage(-4.3);
    m_IntakeRight.setVoltage(4.3);
    m_intakeOn = "normal";
    }

  public void reverseIntake() {
    m_IntakeLeft.setVoltage(6.5);
    m_IntakeRight.setVoltage(-6.5);
    m_intakeOn = "reverse";
    }
  
    public void superCharge() {
    m_IntakeLeft.setVoltage(-5);
    m_IntakeRight.setVoltage(5); // Messing with values for 2" - Langgang
    m_intakeOn = "super charge";
    }


  /**
   * Method used to stop intake
   * 
   * @author Larry9297
   */
  public void stopIntake() {
    m_IntakeLeft.setVoltage(0);
    m_IntakeRight.setVoltage(0);
    m_intakeOn = "off";
  }


}