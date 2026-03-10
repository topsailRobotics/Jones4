// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
//larry was here
package frc.robot.subsystems;

import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;

import java.util.logging.Logger;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.ADIS16470_IMU.IMUAxis;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DriveConstants;
import frc.robot.Util.LimelightHelpers;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/*
 * "IMUAxis.kZ" was removed from all versions of m_gyro.getAngle because we use a NavX gyro
 * 
 * 
 * 
 */





public class DriveSubsystem extends SubsystemBase {
  public double aimkp = .0015;
  public double targetingAngularVelocity;
  public double FerryAmount;
  public double setDiamond;
  public double newDiamond;
  public double newAngle;

  // Create MAXSwerveModules
  private final MAXSwerveModule m_frontLeft = new MAXSwerveModule(
      DriveConstants.kFrontLeftDrivingCanId,
      DriveConstants.kFrontLeftTurningCanId,
      DriveConstants.kFrontLeftChassisAngularOffset);

  private final MAXSwerveModule m_frontRight = new MAXSwerveModule(
      DriveConstants.kFrontRightDrivingCanId,
      DriveConstants.kFrontRightTurningCanId,
      DriveConstants.kFrontRightChassisAngularOffset);

  private final MAXSwerveModule m_rearLeft = new MAXSwerveModule(
      DriveConstants.kRearLeftDrivingCanId,
      DriveConstants.kRearLeftTurningCanId,
      DriveConstants.kBackLeftChassisAngularOffset);

  private final MAXSwerveModule m_rearRight = new MAXSwerveModule(
      DriveConstants.kRearRightDrivingCanId,
      DriveConstants.kRearRightTurningCanId,
      DriveConstants.kBackRightChassisAngularOffset);

  // The gyro sensor
private final AHRS m_gyro = new AHRS(NavXComType.kUSB1);
private SwerveDrivePoseEstimator m_poseEstimator;

private final Field2d m_field = new Field2d();

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    // Usage reporting for MAXSwerve template
    HAL.report(tResourceType.kResourceType_RobotDrive, tInstances.kRobotDriveSwerve_MaxSwerve);

    // Initialize Pose Estimator
    m_poseEstimator = new SwerveDrivePoseEstimator(
        DriveConstants.kDriveKinematics,
        Rotation2d.fromDegrees(-(m_gyro.getAngle()) + 180),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        },
        new Pose2d());

    SmartDashboard.putData("Field", m_field);
  }

  @Override
  public void periodic() {
    //aiming at hub code
    targetingAngularVelocity = LimelightHelpers.getTX("limelight-four") * aimkp;
    targetingAngularVelocity *= DriveConstants.kMaxAngularSpeed;
    //System.out.println(m_gyro.getAngle());
    //invert since tx is positive when the target is to the right of the crosshair
    targetingAngularVelocity *= -1.0;
    if (LimelightHelpers.getTX("limelight-four") <= 3 && LimelightHelpers.getTX("limelight-four") >= -3){
      targetingAngularVelocity = 0;
    }
    // ferrying angle
    newAngle = (m_gyro.getAngle()-180) % 360;
    if (newAngle >= 180){
      newAngle = -(180-(newAngle - 180));
    } else if(newAngle <= -180){
      newAngle = 180-(newAngle + 180);
    }
    if (newAngle >= 1 || newAngle <= -1){
      FerryAmount = (((newAngle)) * 0.005);
      if (FerryAmount >= .5){
        FerryAmount = .5;
      } else if( FerryAmount <=-0.5){
        FerryAmount = -.5;
      }
    } else {
      FerryAmount = 0;
    }
    //driving over bump optimal angle
    newDiamond = (m_gyro.getAngle()-45) % 90;
    if (newDiamond >= 45){
      newDiamond = -(45-(newAngle - 45));
    } else if(newDiamond <= -45){
      newDiamond = 45-(newAngle + 45);
    }
    if (newDiamond >= 10 || newDiamond <= -10){
      setDiamond = (((newDiamond)) * 0.005);
      if (setDiamond >= .4){
        setDiamond = .4;
      } else if( setDiamond <=-0.5){
        setDiamond = -.4;
      }
    } else {
      setDiamond = 0;
    }

    

    m_poseEstimator.update(
        getHeading(),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        });
     //poseEstimation using megatag1 and 2
    if (m_poseEstimator == null || m_gyro == null) { //prevent null pointer exception in disabled periodic
      return;
    }
    
    LimelightHelpers.SetRobotOrientation("limelight-four", m_poseEstimator.getEstimatedPosition().getRotation().getDegrees(), 0, 0, 0, 0, 0);
    var mt2_visionEstimate = 
        LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-four");
    var visionEstimate =
        LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight-four");//use limelight host name
    
    // if our angular velocity is greater than 360 degrees per second, ignore vision updates
    //rejection booleans
    boolean mt2_reject = false;
    boolean reject = false;
    
    //mt2
    if (mt2_visionEstimate == null || mt2_visionEstimate.tagCount == 0) {
      mt2_reject = true;
    }
    if(Math.abs(m_gyro.getRate()) > 360)
    {
      mt2_reject = true;
    }
    
    if(!mt2_reject)
    {
      m_poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(.7,.7,9999999));
      m_poseEstimator.addVisionMeasurement(
          mt2_visionEstimate.pose,
          mt2_visionEstimate.timestampSeconds);
    }
  //mt1
  if(mt2_reject)
  {
    if (visionEstimate == null || visionEstimate.tagCount == 0) { //prevent null initialization, short-circuits
      reject = true;
    }

    if (visionEstimate.tagCount == 1 &&
               visionEstimate.rawFiducials.length == 1) {
      if (visionEstimate.rawFiducials[0].ambiguity > 0.7) {
        reject = true;
      }
      if (visionEstimate.rawFiducials[0].distToCamera > 3) {
        reject = true;
      }
    }

    if (!reject) {
      m_poseEstimator.addVisionMeasurement(
          visionEstimate.pose,
          visionEstimate.timestampSeconds);
    }
  }
 m_field.setRobotPose(m_poseEstimator.getEstimatedPosition());//update robot position in field, further reviews needed
  SmartDashboard.putNumber("NavX Gyro", m_gyro.getAngle());
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_poseEstimator.getEstimatedPosition();
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    m_poseEstimator.resetPosition(
        getHeading(),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        },
        pose);
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed        Speed of the robot in the x direction (forward).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param rot           Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the
   *                      field.
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    // Convert the commanded speeds into the correct units for the drivetrain
    double xSpeedDelivered = xSpeed * DriveConstants.kMaxSpeedMetersPerSecond;
    double ySpeedDelivered = ySpeed * DriveConstants.kMaxSpeedMetersPerSecond;
    double rotDelivered = rot * DriveConstants.kMaxAngularSpeed;

    var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered,
                getHeading())
            : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered));
    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_rearLeft.setDesiredState(swerveModuleStates[2]);
    m_rearRight.setDesiredState(swerveModuleStates[3]);
  }

  /**
   * Sets the wheels into an X formation to prevent movement.
   */
  public void setX() {
    m_frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    m_rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    m_rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    System.out.println("setx");
  }

  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(desiredStates[0]);
    m_frontRight.setDesiredState(desiredStates[1]);
    m_rearLeft.setDesiredState(desiredStates[2]);
    m_rearRight.setDesiredState(desiredStates[3]);
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    m_frontLeft.resetEncoders();
    m_rearLeft.resetEncoders();
    m_frontRight.resetEncoders();
    m_rearRight.resetEncoders();
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    m_gyro.reset();
  }

  /**
   * Returns the heading of the robot.
   * @return the robot's heading, type: Rotational2d
   */
  public Rotation2d getHeading() {
    return Rotation2d.fromDegrees(-m_gyro.getAngle());
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return m_gyro.getRate() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }
}