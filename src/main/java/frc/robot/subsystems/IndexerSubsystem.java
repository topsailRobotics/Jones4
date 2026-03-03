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
  /** Creates a new ExampleSubsystem. */
  private final SparkMax m_hori = new SparkMax(IndexerConstants.kbeltID, MotorType.kBrushless);
  private final SparkMax m_vert= new SparkMax(IndexerConstants.kwheelVertID, MotorType.kBrushless);
  private final SparkClosedLoopController m_pidController1 = m_hori.getClosedLoopController();
  private SparkAnalogSensor m_encoder = m_hori.getAnalog(); 
//   private SparkAbsoluteEncoder m_encoder = m_hori.getAbsoluteEncoder(); 
//   ^^^ for using the absolute encoder, should theoretically work


  public IndexerSubsystem() {}

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
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

  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
  public void runIndex() { //potentially add threshold for horizontal belt activation - carter + larry
        m_hori.setVoltage(8);
        m_vert.setVoltage(-8);
  }
  public void stopIndex() {
    m_hori.setVoltage(0);
    m_vert.setVoltage(0);
  }


}
