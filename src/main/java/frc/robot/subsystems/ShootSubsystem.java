// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkAnalogSensor;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import frc.robot.Constants.ShooterConstants;

public class ShootSubsystem extends SubsystemBase {
 
    // front: intake
    // back: electricals
    // left: climber
    // right: battery
  private final SparkMax m_ShootLeft = new SparkMax(ShooterConstants.kleftshootermotorID, MotorType.kBrushless);
  private final SparkMax m_ShootRight= new SparkMax(ShooterConstants.krightshootermotorID, MotorType.kBrushless);
 
  //default construcor
  public ShootSubsystem() {

  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  //For now, use 4 distinct outputs based on distance ranges from the hub. 
  //Use shooterTest for testing

  public void shooterTest()
  {
        // add constants for voltage setpoint later
        m_ShootLeft.setVoltage(-7);
        m_ShootRight.setVoltage(7);
  }
  public void shooterMedium()
  {
        // add constants for voltage setpoint later
        m_ShootLeft.setVoltage(5);
        m_ShootRight.setVoltage(-5);
  }
  public void shooterHigh()
  {
        // add constants for voltage setpoint later
        m_ShootLeft.setVoltage(6);
        m_ShootRight.setVoltage(-6);
  }
  public void shooterMax()
  {
        // add constants for voltage setpoint later
        m_ShootLeft.setVoltage(7);
        m_ShootRight.setVoltage(-7);
  }

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
