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
//import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.Constants.IntakeConstants;     currently unused
//import edu.wpi.first.wpilibj.Timer;

/**
 * Class that is used to command the robot to shoot.
 * 
 * @author ziwei8658
 */
public class Shoot extends Command {

  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})  //remove later
  

  //instance variables
  private final ShootSubsystem m_shoot;
  //private DriveSubsystem m_drive;
  private int range;
  //private final Timer timer = new Timer();

  /**
   * The constructor for the Shoot class.
   * 
   * @param m_intake UNKNOWN PURPOSE OF PARAMETER (PLEASE FILL IN DOCUMENTATION)
   * @param m_shoot UNKNOWN PURPOSE OF PARAMETER (PLEASE FILL IN DOCUMENTATION)
   * @author ziwei8658
   */
  public Shoot(ShootSubsystem m_shoot,int range) {
    this.m_shoot = m_shoot;
    this.range = range;
    addRequirements(m_shoot); //declare exclusive subsystem control
  }

  /*
   * methods
   */

  /**
   * Called when the intake command is initially scheduled to confirm that the command has been initialized.
   * 
   * @author ziwei8658
   */
  @Override     // Called when the command is initially scheduled.
  public void initialize() {
    System.out.println("shoot initialized");
    
  }

  /**
   * Release intake and run internal wheels. Called every time the scheduler runs while the shoot command is scheduled.
   * 
   * @author ziwei8658
   */
  @Override
  public void execute() {
    /* 
    range = m_drive.getRange();
    if(range>=5) //specific ranges to be changed
    {
      m_shoot.shooterHigh();
    }else if(range>=4)
    {
      m_shoot.shooterMedium();
    }else if(range>=3)
    {
      m_shoot.shooterLow();
    }
    */
    if(range == 1)
    {
       m_shoot.shooterLow();
    } else if (range == 2){
      m_shoot.shooterMediumLow();
    } else  if(range==3){
       m_shoot.shooterMedium();
    }else if(range==4)
    {
       m_shoot.shooterHigh();
    }
    /*if(m_shoot.m_Encoder.getVelocity()<=-2445){
      m_indexer.runIndexVert();
      } */
    // m_indexer.runIndexVert();
    // m_indexer.runIndexHori();
    // m_shoot.shooterTest();
  }
  
  @Override  // Called once the command ends or is interrupted.
  public void end(boolean interrupted) {
    m_shoot.stopShooter();
    System.out.println("shooter off");
  }

  /**
   * Returns true when the command should end. Does not end on its own, the command is ended when external schedules are false(see robot container)
   * 
   * @return Will return false when called.
   * @author ziwei8658
   */
  @Override
  public boolean isFinished() {
    return false;

  }

}//end of class
