// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  /**
   * Constants used for being able to use controllers
   */
  public static class OperatorConstants {
    public static final int kDriverControllerPort1 = 0;
    public static final int kDriverControllerPort2 = 1;
  }

  /**
   * Constant values used for driving accross entire robot.
   */
  public static final class DriveConstants {
    // Driving Parameters - Note that these are not the maximum capable speeds of
    // the robot, rather the allowed maximum speeds
    public static final double kMaxSpeedMetersPerSecond = 6.01; //7.60 based on documents but need to be sure - Willem 
    // changed it to 24.93 (feet)
    public static final double kMaxAngularSpeed = 2 * Math.PI; // radians per second was 8 pi for comp

    // Chassis configuration
    public static final double kTrackWidth = Units.inchesToMeters(26.5);
    // Distance between centers of right and left wheels on robot
    public static final double kWheelBase = Units.inchesToMeters(21.8);
    // Distance between front and back wheels on robot
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

    // Angular offsets of the modules relative to the chassis in radians
    public static final double kFrontLeftChassisAngularOffset = -Math.PI / 2;
    public static final double kFrontRightChassisAngularOffset = 0;
    public static final double kBackLeftChassisAngularOffset = Math.PI;
    public static final double kBackRightChassisAngularOffset = Math.PI / 2;

    // SPARK MAX CAN IDs
    public static final int kFrontLeftDrivingCanId = 11; //OG was 2
    public static final int kRearLeftDrivingCanId = 44; //OG was 4
    public static final int kFrontRightDrivingCanId = 22; //OG was 1
    public static final int kRearRightDrivingCanId = 33; //OG was 3


    public static final int kFrontLeftTurningCanId = 1; //OG was 22
    public static final int kRearLeftTurningCanId = 4; //OG was 44
    public static final int kFrontRightTurningCanId = 2; //OG was 11
    public static final int kRearRightTurningCanId = 3; //OG was 33

    public static final boolean kGyroReversed = false;
    
  }

  public static final class BlinkinConstants {
    public static final double Gold = 0.67;
    public static final double Black = 0.99;
    public static final double Blue = .87;
    public static final double Blue_Violet = .89;
    public static final double Violet = .91;
    public static final double Hot_Pink = .57;
    public static final double Dark_Red = .59;
    public static final double Red = -.61;
    public static final double Green = .77;
    public static final double White = .93;

    // testing /cool values
    public static final double Rainbowlava = -.93;
    public static final double Confetti = -.87;
    public static final double Heartbeatblue = -.25;
    public static final double Heartbeatred = -.23;
    public static final double Breathslow = .29;
    public static final double Breathfast = .31;
    public static final double Strobe = .35;
    
  }
  /**
   * Constants purpose is unkown. This is currently used in configs file exclusively.
   */
  public static final class ModuleConstants {
    // The MAXSwerve module can be configured with one of three pinion gears: 12T,
    // 13T, or 14T. This changes the drive speed of the module (a pinion gear with
    // more teeth will result in a robot that drives faster).
    public static final int kDrivingMotorPinionTeeth = 14;
    public static final int kDrivingMotorDrivenTeeth = 21; //high reduction --- changed this to 16 for the new gear ratio - Willem

    // Calculations required for driving motor conversion factors and feed forward
    public static final double kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60;
    public static final double kWheelDiameterMeters = 0.0762;
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
    // 45 teeth on the wheel's bevel gear, 20 teeth on the first-stage spur gear, 16
    // teeth on the bevel pinion

    
    public static final double kDrivingMotorReduction = (45.0 * kDrivingMotorDrivenTeeth) / (kDrivingMotorPinionTeeth * 15);
    public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters)
        / kDrivingMotorReduction;
  }

  /**
   * To be honest I have no clue why this class exists since its only recorded use is in another constants class.
   */
  public static final class NeoMotorConstants {
    public static final double kFreeSpeedRpm = 6784;//changed this to the neo vortex speed 
  }

  /**
   * UNKOWN PURPOSE OF CLASS (PLEASE FILL IN DEV DOCS)
   */
  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final double kDriveDeadband = 0.05;
  }

  /**
    * Constants used in aligning the robot to the hub via apriltags.
  */

  /**
   * Constants used in the intake process.
  */
  public static final class IntakeConstants { //change the constants
    public static final int kIntakeWheelID = 51;
    public static final int kIntakeArmID = 52;
    public static final double kIntakeSetpoint = 3;
  }

  /**
   * Constants used for the CAN IDs of the wheels in the index (system that cycles balls thorugh the center of the robot to eventually shoot them out).
   */
  public static final class IndexerConstants{ //change IDS
    public static final int kbeltID = 53; 
    public static final int kwheelVertID = 54; 
  }

  /**
   * constants used to help calulate the shooter to aim correctly. ALso has CAN ID values for shooter motors.
   */
  public static final class ShooterConstants{
    public static final double shooterHeight = 0; //in meters
    public static final double targetHeight = 0; //in meters
    public static final double netHeight = targetHeight - shooterHeight;
    public static final double lockedAngle = 87.088; //in degrees
    public static final double relationModification = 1; //no unit; modifies ball exit velocity to wheel spin speed
    public static final int kleftshootermotorID = 55; 
    public static final int krightshootermotorID = 56; 

  }

}