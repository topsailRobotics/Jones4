package frc.robot.subsystems;

//import java.io.ObjectInputFilter.Config;

//import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.FeedbackSensor;
//import com.revrobotics.spark.SparkAnalogSensor;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkMax;
//import com.revrobotics.spark.SparkBase.ControlType;
//import com.revrobotics.spark.SparkBase.PersistMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import edu.wpi.first.units.measure.Velocity;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
public class IntakeSubsystem extends SubsystemBase {
      public String m_intakeOn = "Off";

  /** Creates a new ExampleSubsystem. */

  // Initilization
private final SparkMax m_IntakeLeft = new SparkMax(IntakeConstants.kIntakeArmID, MotorType.kBrushless);
private final SparkMax m_IntakeRight= new SparkMax(IntakeConstants.kIntakeWheelID, MotorType.kBrushless);

//private final SparkClosedLoopController m_pidController1 = m_IntakeArm.getClosedLoopController();
//private final AbsoluteEncoder m_AbsoluteEncoder = m_IntakeArm.getAbsoluteEncoder();
  
public IntakeSubsystem() {
  /* 
SparkMaxConfig config = new SparkMaxConfig();
config.closedLoop.
feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
.pid(1, 0.0025, 0.05)
.outputRange(-1,1);
m_IntakeArm.configure(config,ResetMode.kResetSafeParameters, com.revrobotics.PersistMode.kPersistParameters);
  */
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   * 
   * @author Larry9297
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

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
    m_IntakeLeft.setVoltage(-5.5);
    m_IntakeRight.setVoltage(5.5);
    m_intakeOn = "SUPERCHARGE";
    }


  /**
   * Method used to stop intake
   * 
   * @author Larry9297
   */
  public void stopIntake() {
    m_IntakeLeft.setVoltage(0);
    m_IntakeRight.setVoltage(0);
    m_intakeOn = "Off";
  }

  //intake out will be scheduled separately using on true logic
  public void intakeUp(double setposition)
  {
    //set point .15
    //m_pidController1.setSetpoint(setposition, com.revrobotics.spark.SparkBase.ControlType.kDutyCycle);
  }
  
  //default state is set point 0, parameter omitted
  public void intakeOff()
  {
    //m_pidController1.setSetpoint(0, com.revrobotics.spark.SparkBase.ControlType.kDutyCycle); //0.85 is at 0ish
  }

  

}