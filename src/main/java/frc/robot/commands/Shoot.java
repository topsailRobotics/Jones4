// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/*
 * this command file combines actions from indexer subsystem and intake subsystem for internal fuel storage
 * shootersubsystem methods may be incorporated later
 * will be used in auto
 * this command only controls wheels spinning
 */

//try having timer in execute only for indexer to make it wait
//imports
package frc.robot.commands;
import frc.robot.subsystems.ShootSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;

public class Shoot extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})  //remove later
  
  //instance variables
  private final IndexerSubsystem m_indexer;
  private final ShootSubsystem m_shoot;
  
  /*
   * constructor
   */
  public Shoot(IndexerSubsystem m_indexer, ShootSubsystem m_shoot) {
    this.m_indexer = m_indexer;
    this.m_shoot = m_shoot;
    addRequirements(m_indexer, m_shoot); //declare exclusive subsystem control
  }

  /*
   * methods
   */
  @Override     // Called when the command is initially scheduled.
  public void initialize() {
    System.out.println("shoot initialized");
    
  }
  //release intake and run internal wheels
  @Override  // Called every time the scheduler runs while the command is scheduled.
  public void execute() {
    m_indexer.runIndexVert();
    m_indexer.runIndexHori();
    m_shoot.shooterTest();
  }
  
  @Override  // Called once the command ends or is interrupted.
  public void end(boolean interrupted) {
    m_indexer.stopIndexVert();
    m_indexer.stopIndexHori();
    m_shoot.stopShooter();
  }

  // Returns true when the command should end.
  //does not end on its own, the command is ended when external schedules are false(see robot container)
  @Override
  public boolean isFinished() {
    return false;
  }

}//end of class
