// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import com.revrobotics.spark.SparkAnalogSensor;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;


public class ShooterSubsystem extends SubsystemBase {

  private final SparkMax m_shooterMotorLeft = new SparkMax(Constants.ShooterConstants.kleftshootermotorID, MotorType.kBrushless);
  private final SparkMax m_shooterMotorRight = new SparkMax(Constants.ShooterConstants.krightshootermotorID, MotorType.kBrushless);
  private final SparkClosedLoopController m_pidController1 = m_shooterMotorLeft.getClosedLoopController();

  public ShooterSubsystem() {}

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

  public double CalculateWheelVelocity(double distance) {
    double output = (96.04 * distance * distance * (Math.atan(Math.toRadians(Constants.ShooterConstants.lockedAngle)))) + (96.04 * distance) + (-9.8 * Constants.ShooterConstants.netHeight);
    output /= 2;
    return Constants.ShooterConstants.relationModification * Math.pow(output,0.25);
  }

  public void runShooter(double setposition){
    m_shooterMotorLeft.setVoltage(4);
    m_shooterMotorRight.setVoltage(4);
    m_pidController1.setSetpoint(setposition, com.revrobotics.spark.SparkBase.ControlType.kPosition);
  }

  public void stopShooter(double setposition){
    m_shooterMotorLeft.setVoltage(0);
    m_shooterMotorRight.setVoltage(0);
    m_pidController1.setSetpoint(setposition, com.revrobotics.spark.SparkBase.ControlType.kPosition);
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
