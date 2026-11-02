// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkFlex;
import frc.robot.Configs;
import frc.robot.Constants.ModuleConstants;

import com.revrobotics.ResetMode;
import com.revrobotics.PersistMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.FeedbackSensor;

public class MAXSwerveModule {
  private final SparkFlex m_drivingSpark;
  private final SparkMax m_turningSpark;

  private final SparkFlexConfig m_drivingConfig = new SparkFlexConfig();
  private final SparkMaxConfig m_turningConfig = new SparkMaxConfig();

  private final RelativeEncoder m_drivingEncoder;
  private final AbsoluteEncoder m_turningEncoder;

  private final SparkClosedLoopController m_drivingClosedLoopController;
  private final SparkClosedLoopController m_turningClosedLoopController;

  private double m_chassisAngularOffset = 0;
  private SwerveModuleState m_desiredState = new SwerveModuleState(0.0, new Rotation2d());

  /**
   * Constructs a MAXSwerveModule and configures the driving and turning motor,
   * encoder, and PID controller. This configuration is specific to the REV
   * MAXSwerve Module built with NEOs, SPARKS MAX, and a Through Bore
   * Encoder.
   * 
   * @param drivingCANId The CAN id that is associated with the turning of the wheels on the roll axis on a swerve module.
   * @param turningCANId The CAN id that is associated with the turning on the yaw axis of the wheels on a swerve module.
   * @param chassisAngularOffset UNKOWN PURPOSE OF PARAMETER (PLEASE FILL IN DOCUMENTATION)
   * 
   * @author septiceyesam2049-bot
   */
  public MAXSwerveModule(int drivingCANId, int turningCANId, double chassisAngularOffset) {

    m_drivingSpark = new SparkFlex(drivingCANId, MotorType.kBrushless);
    m_turningSpark = new SparkMax(turningCANId, MotorType.kBrushless);

    m_drivingEncoder = m_drivingSpark.getEncoder();
    m_turningEncoder = m_turningSpark.getAbsoluteEncoder();

    m_drivingClosedLoopController = m_drivingSpark.getClosedLoopController();
    m_turningClosedLoopController = m_turningSpark.getClosedLoopController();

    // ==========================================
    // DRIVING MOTOR CONFIGURATION (SPARK FLEX)
    // ==========================================

    double drivingFactor =
        ModuleConstants.kWheelDiameterMeters * Math.PI
        / ModuleConstants.kDrivingMotorReduction;

    double drivingP = 0.04 * drivingFactor / 60.0;

    double drivingKV =
        12.0 * drivingFactor
        / (60.0 * ModuleConstants.kDriveWheelFreeSpeedRps);

    SparkFlexConfig drivingConfig = new SparkFlexConfig();

    drivingConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(50);

    drivingConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pid(drivingP, 0, 0)
        .outputRange(-1, 1)
        .feedForward.kV(drivingKV);


    // ==========================================
    // TURNING MOTOR CONFIGURATION (SPARK MAX)
    // ==========================================

    SparkMaxConfig turningConfig = new SparkMaxConfig();

    turningConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(20);

    turningConfig.absoluteEncoder
        .inverted(true);

    turningConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
        .pid(2 * Math.PI, 0, 0)
        .outputRange(-1, 1)
        .positionWrappingEnabled(true)
        .positionWrappingInputRange(0, 1);


    // ==========================================
    // APPLY CONFIGURATIONS
    // ==========================================

    m_drivingSpark.configure(
        drivingConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters
    );

    m_turningSpark.configure(
        turningConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters
    );


    // ==========================================
    // INITIALIZE MODULE
    // ==========================================

    m_chassisAngularOffset = chassisAngularOffset;

    m_desiredState.angle = new Rotation2d(
        m_turningEncoder.getPosition() * 2 * Math.PI
    );

    m_drivingEncoder.setPosition(0);
}

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   * 
   * @author septiceyesam2049-bot
   */
  public SwerveModuleState getState() {
    // Apply chassis angular offset to the encoder position to get the position
    // relative to the chassis.
    return new SwerveModuleState(m_drivingEncoder.getVelocity(),
        new Rotation2d(m_turningEncoder.getPosition() - m_chassisAngularOffset));
  }

  /**
   * Returns the current position of the module.
   *
   * @return The current position of the module.
   * 
   * @author septiceyesam2049-bot
   */
  public SwerveModulePosition getPosition() {
    // Apply chassis angular offset to the encoder position to get the position
    // relative to the chassis.
    return new SwerveModulePosition(
        m_drivingEncoder.getPosition(),
        new Rotation2d(m_turningEncoder.getPosition() - m_chassisAngularOffset));
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   * 
   * @author septiceyesam2049-bot
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    // Apply chassis angular offset to the desired state.
    SwerveModuleState correctedDesiredState = new SwerveModuleState();
    correctedDesiredState.speedMetersPerSecond = desiredState.speedMetersPerSecond;
    correctedDesiredState.angle = desiredState.angle.plus(Rotation2d.fromRadians(m_chassisAngularOffset));

    // Optimize the reference state to avoid spinning further than 90 degrees.
    correctedDesiredState.optimize(new Rotation2d(m_turningEncoder.getPosition()));

    // Command driving and turning SPARKS towards their respective setpoints.
    m_drivingClosedLoopController.setReference(correctedDesiredState.speedMetersPerSecond, ControlType.kVelocity);
    m_turningClosedLoopController.setReference(correctedDesiredState.angle.getRadians(), ControlType.kPosition);

    m_desiredState = desiredState;
  }

  /** 
   * Zeroes all the SwerveModule encoders. 
   * 
   * @author septiceyesam2049-bot
  */
  public void resetEncoders() {
    m_drivingEncoder.setPosition(0);
  }
}