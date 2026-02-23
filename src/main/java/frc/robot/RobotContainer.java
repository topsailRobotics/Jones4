// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
//hi
package frc.robot;

import frc.robot.Constants.OIConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Util.LimelightHelpers;
import frc.robot.commands.Autos;
import frc.robot.commands.Climb;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  //private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private final IntakeSubsystem m_intake = new IntakeSubsystem();
  private final IndexerSubsystem m_indexer = new IndexerSubsystem();
  private final ClimberSubsystem m_ClimberSubsystem = new ClimberSubsystem();



  // Replace with CommandPS4Controller or CommandJoystick if needed
  //This is the driver's controller
  private final CommandXboxController m_driverController0 =
      new CommandXboxController(OperatorConstants.kDriverControllerPort1);
  //This is the operator's controller
  private final CommandXboxController m_driverController1 =
      new CommandXboxController(OperatorConstants.kDriverControllerPort2);


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
  //  new Trigger(m_exampleSubsystem::exampleCondition)
  //      .onTrue(new ExampleCommand(m_exampleSubsystem));

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
  //  m_driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());
      m_robotDrive.setDefaultCommand(
          // The left stick controls translation of the robot.
          // Turning is controlled by the X axis of the right stick.
          new RunCommand(
              () -> m_robotDrive.drive(
                  -MathUtil.applyDeadband(m_driverController0.getLeftY(), OIConstants.kDriveDeadband),
                  -MathUtil.applyDeadband(m_driverController0.getLeftX(), OIConstants.kDriveDeadband),
                  -MathUtil.applyDeadband(m_driverController0.getRightX(), OIConstants.kDriveDeadband),
                  true),
              m_robotDrive));
      m_intake.setDefaultCommand(new RunCommand(()-> m_intake.stopIntake(0),m_intake));
      m_indexer.setDefaultCommand(new RunCommand(()-> m_indexer.stopIndex(),m_indexer));
      m_ClimberSubsystem.setDefaultCommand(new RunCommand(()-> m_ClimberSubsystem.stopClimber(),m_ClimberSubsystem));

      



    Trigger aimingTrigger = new Trigger (()-> m_driverController0.getLeftTriggerAxis() > 0.5 && LimelightHelpers.getFiducialID("limelight-second")>=0);
    aimingTrigger.whileTrue( new RunCommand(
              () -> m_robotDrive.drive(
                  -MathUtil.applyDeadband(m_driverController0.getLeftY(), OIConstants.kDriveDeadband),
                  -MathUtil.applyDeadband(m_driverController0.getLeftX(), OIConstants.kDriveDeadband),
                  m_robotDrive.targetingAngularVelocity,
                  true),
              m_robotDrive));
    
    Trigger ferryTrigger = new Trigger(() -> m_driverController0.getRightTriggerAxis() > 0.5);
    ferryTrigger.whileTrue( new RunCommand(
              () -> m_robotDrive.drive(
                  -MathUtil.applyDeadband(m_driverController0.getLeftY(), OIConstants.kDriveDeadband),
                  -MathUtil.applyDeadband(m_driverController0.getLeftX(), OIConstants.kDriveDeadband),
                  m_robotDrive.FerryAmount,
                  true),
              m_robotDrive));
    
   
    m_driverController0.rightBumper().whileTrue( new RunCommand(
              () -> m_robotDrive.drive(
                  -MathUtil.applyDeadband(m_driverController0.getLeftY(), OIConstants.kDriveDeadband),
                  -MathUtil.applyDeadband(m_driverController0.getLeftX(), OIConstants.kDriveDeadband),
                  m_robotDrive.setDiamond,
                  true),
              m_robotDrive));
    m_driverController0.leftBumper().toggleOnTrue(new RunCommand(
              () -> m_robotDrive.setX()));
    
    //true for climer up, false for down, independent commands sharing same command file
    m_driverController0.a().onTrue(new Climb(m_ClimberSubsystem, true));
    m_driverController0.b().onTrue(new Climb(m_ClimberSubsystem, false));
    
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
// public Command climbCommand(){
//    return Commands.startEnd(
//         () -> {
//           Commands.run(()-> new Climb(m_ClimberSubsystem, true));
//           System.out.println("climber up");
//         },
//         () -> {
//           Commands.run(()-> new Climb(m_ClimberSubsystem, false));
//           System.out.println("climber down");
//         });
//   public Command getAutonomousCommand() {
//     // An example command will be run in autonomous
//     //return Autos.exampleAuto(m_exampleSubsystem);
//   }
  
}//end of class
