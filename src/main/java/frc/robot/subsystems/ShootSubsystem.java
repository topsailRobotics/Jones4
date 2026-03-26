// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

//import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.RelativeEncoder;
//import com.revrobotics.spark.SparkAnalogSensor;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;

import frc.robot.Constants.ShooterConstants;

public class ShootSubsystem extends SubsystemBase {
 
    // front: intake
    // back: electricals
    // left: climber
    // right: battery
  private final SparkMax m_ShootLeft = new SparkMax(ShooterConstants.kleftshootermotorID, MotorType.kBrushless);
  private final SparkMax m_ShootRight= new SparkMax(ShooterConstants.krightshootermotorID, MotorType.kBrushless);
  public final RelativeEncoder m_Encoder = m_ShootLeft.getEncoder();
private final SparkClosedLoopController m_pidController1 = m_ShootLeft.getClosedLoopController();
private final SparkClosedLoopController m_pidController2 = m_ShootRight.getClosedLoopController();
  //default construcor
  public ShootSubsystem() {

  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  //For now, use 4 distinct outputs based on distance ranges from the hub. 
  //Use shooterTest for testing

/**
 *smart shoot method
 *try interpolation and wpilib mapping later
 *@param distance : in meters between robot and 
 *{@link https://firstfrc.blob.core.windows.net/frc2026/FieldAssets/2026-field-dimension-dwgs.pdf}
 */
public double getShooterRPM(double distance)
{
  //default case
  if (distance == -1) return 0;
  //simple interpolation
  if (distance >= 1 && distance <=4 )
  {
    return 2500 + 300 * (distance-1);
  }
  //ferry, when distance too far, most likely unused
  return 5000;
}

public void smartShoot(double rpm)
{
  m_pidController1.setSetpoint(-rpm, ControlType.kVelocity);
  m_pidController2.setSetpoint(-rpm, ControlType.kVelocity);

}


 /**
   * Method used to test the shooter. Will probably get deleted or not be used in final robot.
   * 
   * @author Carter
 */
  public void shooterTest()
  {
      m_pidController1.setSetpoint(-2500, ControlType.kVelocity);
      m_pidController2.setSetpoint(-2500, ControlType.kVelocity);
      System.out.println(m_Encoder.getVelocity());// add constants for voltage setpoint later
  }
  public void shooterLow()
  {
      m_pidController1.setSetpoint(-2500, ControlType.kVelocity); // straight in front of the hub
      m_pidController2.setSetpoint(-2500, ControlType.kVelocity);

  }

    public void shooterMediumLow()
  {
      m_pidController1.setSetpoint(-3100, ControlType.kVelocity); //
      m_pidController2.setSetpoint(-3100, ControlType.kVelocity);
  }


  
  public void shooterMedium()
  {
      m_pidController1.setSetpoint(-3300, ControlType.kVelocity); //
      m_pidController2.setSetpoint(-3300, ControlType.kVelocity);
  }

   /**
    * Method used to set shooter to the medium distance.
    * 
    * @author Carter
   */
  public void shooterHigh()
  {
        // add constants for voltage setpoint later
      m_pidController1.setSetpoint(-5000, ControlType.kVelocity); //ferry 18-16 feet
      m_pidController2.setSetpoint(-5000, ControlType.kVelocity);
  }

  /**
    * Method used to set shooter to the maximum distance.
    * 
    * @author Carter
   */
  public void shooterMax()
  {
        // add constants for voltage setpoint later
        m_ShootLeft.setVoltage(-7);
        m_ShootRight.setVoltage(7);
  }

  /**
    * Method used to stop the shooter.
    * 
    * @author Carter
   */
   public void stopShooter()
  {
        // add constants for voltage setpoint later
        m_ShootLeft.setVoltage(0);
        m_ShootRight.setVoltage(0);
  }
  
  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
