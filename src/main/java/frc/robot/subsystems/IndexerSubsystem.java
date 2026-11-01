// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;

//import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IndexerConstants;


import com.revrobotics.ResetMode;
import com.revrobotics.PersistMode;

import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

public class IndexerSubsystem extends SubsystemBase {
  private final SparkMax m_vert= new SparkMax(IndexerConstants.kwheelVertID, MotorType.kBrushless);
  private final SparkClosedLoopController m_pidController1 = m_vert.getClosedLoopController();
  private final SparkMaxConfig m_vertConfig = new SparkMaxConfig();

  //Add constructor and put the specific configs here for initialization
  public IndexerSubsystem() {

    m_vertConfig
        .idleMode(IdleMode.kBrake);
    m_vertConfig
        .smartCurrentLimit(40);
    m_vert.configure(
    m_vertConfig, //apply the config, the other 2 arguments are kinda just there...
    ResetMode.kResetSafeParameters,
    PersistMode.kPersistParameters
);

}
  /**
   * Method to run the vertical indexer.
   * 
   * @author UNKOWN
   */
  public void runIndexVert()
  {
   m_pidController1.setSetpoint(1,ControlType.kDutyCycle); //commented out temporarily
  }

  public void reverseIndex(){
    m_pidController1.setSetpoint(-1, ControlType.kDutyCycle);
  }

  /**
   * Method to stop the vertical indexer.
   * 
   * @author UNKOWN
   */
  public void stopIndexVert() {
    m_vert.setVoltage(0);
  }
}
