//password for radio : thsrobotics

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
//hi
package frc.robot;

import frc.robot.Constants.OIConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Util.LimelightHelpers;
import frc.robot.commands.Aim;
import frc.robot.commands.Autos;
import frc.robot.commands.Climb;
import frc.robot.commands.Intake;
import frc.robot.commands.Shoot;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ShootSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

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
  private final ClimberSubsystem m_climber = new ClimberSubsystem();
  private final ShootSubsystem m_shooter = new ShootSubsystem(); //actions not declared yet in shootersubsystem

  //autos
  private final String m_defaultAuto = "New Auto";
  private final String m_TestAuto = "test";

  // A chooser for autonomous commands
  SendableChooser<String> m_chooser = new SendableChooser<>();
  

   ;
    

  // Replace with CommandPS4Controller or CommandJoystick if needed
  //This is the driver's controller
  private final CommandXboxController m_driverController0 =
      new CommandXboxController(OperatorConstants.kDriverControllerPort1);
  //This is the operator's controller
  private final CommandXboxController m_driverController1 =
      new CommandXboxController(OperatorConstants.kDriverControllerPort2);


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Add commands to the autonomous command chooser
  m_chooser.setDefaultOption(m_defaultAuto, m_defaultAuto);
  m_chooser.addOption(m_TestAuto, m_TestAuto);
  m_chooser.addOption(m_TestAuto, m_TestAuto);
  m_chooser.addOption(m_TestAuto, m_TestAuto);

  // Put the chooser on the dashboard
  SmartDashboard.putData(m_chooser);

    NamedCommands.registerCommand("ShootLow", new Shoot(m_indexer,m_shooter,1));
    NamedCommands.registerCommand("ShootMedium", new Shoot(m_indexer,m_shooter,2));
    NamedCommands.registerCommand("ShootHigh", new Shoot(m_indexer,m_shooter,3));
    NamedCommands.registerCommand("Intake", new Intake(m_intake,m_indexer, false));
    //NamedCommands.registerCommand("ReverseIntake", new Intake(m_intake,m_indexer, true));

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
      m_intake.setDefaultCommand(new RunCommand(()-> {m_intake.intakeOff();m_intake.stopIntake();},m_intake));  //stopIntake method broken up to 2 separate methods, this one controls intaker position
      m_climber.setDefaultCommand(new RunCommand(()-> m_climber.stopClimber(),m_climber));
      m_shooter.setDefaultCommand(new RunCommand(()-> m_shooter.stopShooter(),m_shooter));

      

    Trigger aimingTrigger = new Trigger (()-> m_driverController0.getLeftTriggerAxis() > 0.5 && LimelightHelpers.getFiducialID("limelight-four") > 0);
    aimingTrigger.whileTrue(new RunCommand(
      () -> m_robotDrive.drive(
          m_robotDrive.rangingVelocity,
          -MathUtil.applyDeadband(m_driverController0.getLeftX(), OIConstants.kDriveDeadband),
          m_robotDrive.targetingAngularVelocity,
          false)));
  
    Trigger ferryTrigger = new Trigger(() -> m_driverController0.getRightTriggerAxis() > 0.5);
    ferryTrigger.whileTrue(new RunCommand(
      () -> m_robotDrive.drive(
          -MathUtil.applyDeadband(m_driverController0.getLeftY(), OIConstants.kDriveDeadband),
          -MathUtil.applyDeadband(m_driverController0.getLeftX(), OIConstants.kDriveDeadband),
          m_robotDrive.FerryAmount,
          true)));
    
   
    m_driverController0.rightBumper().whileTrue(new RunCommand(
      () -> m_robotDrive.drive(
          -MathUtil.applyDeadband(m_driverController0.getLeftY(), OIConstants.kDriveDeadband),
          -MathUtil.applyDeadband(m_driverController0.getLeftX(), OIConstants.kDriveDeadband),
          m_robotDrive.setDiamond,
          true)));
      
    m_driverController0.leftBumper().toggleOnTrue(new RunCommand(  //changed from RunCommand to Instant Command, control loop should do the job
              () -> m_robotDrive.setX(),
              m_robotDrive));
    
    //true for climer up, false for down, independent commands sharing same command file
    //m_driverController0.povUp().whileTrue(new Climb(m_climber, true));
    //m_driverController0.povDown().whileTrue(new Climb(m_climber, false));
    
    //x to turn on intake and horizontal indexer to collect and store fuels, click again to turn off
    //y to turn on both indexer and shooter, fuels are pushed into the shooter and launched out
    
    //internal system command
    m_driverController0.x()
    .toggleOnTrue(new Intake(m_intake,m_indexer, false)); //command is scheduled while x is held
  
    m_driverController0.b()
    .toggleOnTrue(new Intake(m_intake, m_indexer, true));
    
    m_driverController0.a().whileTrue(new RunCommand(  //changed from RunCommand to Instant Command, control loop should do the job
              () -> m_intake.intakeUp(0.85),
              m_intake));

    m_driverController0.povDown().onTrue(new InstantCommand(()->m_robotDrive.zeroHeading(), m_robotDrive));


    //shooter commands
    m_driverController1.x()
    .toggleOnTrue(new Shoot(m_indexer,m_shooter,1));
    m_driverController1.y()
    .toggleOnTrue(new Shoot(m_indexer,m_shooter,2));
    m_driverController1.a()
    .toggleOnTrue(new Shoot(m_indexer,m_shooter,3));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // This method loads the auto when it is called, however, it is recommended
    // to first load your paths/autos when code starts, then return the
    // pre-loaded auto/path
    return new PathPlannerAuto(m_chooser.getSelected());
  }

}//end of class
