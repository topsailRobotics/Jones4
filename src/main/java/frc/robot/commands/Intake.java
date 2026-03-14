// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/*
 * this command file combines actions from indexer subsystem and intake subsystem for internal fuel storage
 * shootersubsystem methods may be incorporated later
 * will be used in auto
 * this command only controls wheels spinning
 */

//imports
package frc.robot.commands;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;

public class Intake extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})  //remove later
  
  //instance variables
  private final IntakeSubsystem m_intake;
  private final IndexerSubsystem m_indexer;
  
  /*
   * constructor
   */
  public Intake(IntakeSubsystem m_intake, IndexerSubsystem m_indexer) {
    this.m_intake = m_intake;
    this.m_indexer = m_indexer;
    addRequirements(m_intake); //declare exclusive subsystem control
  }

  /*
   * methods
   */
  @Override     // Called when the command is initially scheduled.
  public void initialize() {
    System.out.println("Intake On");
    
  }
  //release intake and run internal wheels
  @Override  // Called every time the scheduler runs while the command is scheduled.
  public void execute() {
    m_intake.runIntake(); 
   m_intake.intakeUp(0.62);  
   m_indexer.runIndexHori();
  }
  
  @Override  // Called once the command ends or is interrupted.
  public void end(boolean interrupted) {
      m_intake.stopIntake();
      m_intake.intakeOff();
      m_indexer.stopIndexHori();
      System.out.println("Intake off");
  }

  // Returns true when the command should end.
  //does not end on its own, the command is ended when external schedules are false(see robot container)
  @Override
  public boolean isFinished() {
    return false;
  }

}//end of class
