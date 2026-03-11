// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkAnalogSensor;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IndexerConstants;

public class IndexerSubsystem extends SubsystemBase {
  private final SparkMax m_hori = new SparkMax(IndexerConstants.kbeltID, MotorType.kBrushless);
  private final SparkMax m_vert= new SparkMax(IndexerConstants.kwheelVertID, MotorType.kBrushless);
  //pid unused
  private final SparkClosedLoopController m_pidController1 = m_hori.getClosedLoopController();
  private SparkAnalogSensor m_encoder = m_hori.getAnalog(); 
//   private SparkAbsoluteEncoder m_encoder = m_hori.getAbsoluteEncoder(); 
//   ^^^ for using the absolute encoder, should theoretically work

  public void runIndexVert()
  {
    m_vert.setVoltage(5);
  }

   public void runIndexHori()
  {
      m_hori.setVoltage(3);
  }

  public void stopIndexVert() {
    m_hori.setVoltage(0);
  }
  
  public void stopIndexHori() {
    m_vert.setVoltage(0);
  }

}
