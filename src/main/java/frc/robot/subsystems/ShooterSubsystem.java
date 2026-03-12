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
import frc.robot.Constants.ShooterConstants;
import frc.robot.Util.LimelightHelpers;


public class ShooterSubsystem extends SubsystemBase {

  private final SparkMax m_shooterMotorLeft = new SparkMax(Constants.ShooterConstants.kleftshootermotorID, MotorType.kBrushless);
  private final SparkMax m_shooterMotorRight = new SparkMax(Constants.ShooterConstants.krightshootermotorID, MotorType.kBrushless);

  private double distance; //updated later

  public ShooterSubsystem() {
    this.distance = distance;
  }
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

  }

  /**
   * Loops through all april tags on the hubs to try and find the towers to calculate wheel velocity.
   * 
   * Currently returns nothing
   */
  public double CalculateWheelVelocity() { //removed distance parameter, pulls from distance variable
    for (int i = 0; i < ShooterConstants.towerTagID.length; i++){ 
      if (LimelightHelpers.getTV("limelight-second") && LimelightHelpers.getFiducialID("limelight-second") == ShooterConstants.towerTagID[i]){
      double theta = Math.tan(Math.toRadians(LimelightHelpers.getTY("limelight-second")));
      distance = Math.abs(ShooterConstants.towerTagHeight - ShooterConstants.limelightsecondHeight) / theta;
      }
    }
    

    double output = (96.04 * distance * distance * (Math.atan(Math.toRadians(Constants.ShooterConstants.lockedAngle)))) + (96.04 * distance) + (-9.8 * Constants.ShooterConstants.netHeight);
    output /= 2;
    return Constants.ShooterConstants.relationModification * Math.pow(output,0.25);

  }

  /**
   * Method for starting shooter.
   */
  public void runShooter(){ 
    m_shooterMotorLeft.setVoltage(CalculateWheelVelocity());
    m_shooterMotorRight.setVoltage(CalculateWheelVelocity());
  }

  /**
   * Method for stopping shooter.
   */
  public void stopShooter(){ 
    m_shooterMotorLeft.setVoltage(0);
    m_shooterMotorRight.setVoltage(0);
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
