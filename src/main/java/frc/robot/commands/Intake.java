// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

//imports
package frc.robot.commands;
import frc.robot.subsystems.IntakeSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * This command combines actions from indexer subsystem and intake subsystem for internal fuel storage.
 * Shootersubsystem methods may be incorporated later.
 * Will be used in auto.
 * This command controls wheels spinning.
 * 
 * @author ziwei8658
 */
public class Intake extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})  //remove later
  
  //instance variables
  private final IntakeSubsystem m_intake;
  private boolean reverse;

  /**
   * This is the constructor for the Intake class.
   * 
   * @param m_intake UNKNOWN PURPOSE OF PARAMETER (PLEASE FILL IN DOCUMENTATION)
   * @author ziwei8658
   */
  public Intake(IntakeSubsystem m_intake, boolean reverse) {
    this.m_intake = m_intake;
    this.reverse = reverse;
    addRequirements(m_intake); //declare exclusive subsystem control
  }
  /**
   * Called when the intake command is initially scheduled to confirm that the command has been initialized.
   * 
   * @author ziwei8658
   */
  @Override     
  public void initialize() {  
    System.out.println("Intake");  
  }
  /**
   * Releases intake and runs internal wheels. This is called every time the scheduler runs while the command is scheduled.
   * 
   * @author ziwei8658
   */
  @Override  
  public void execute() {
    if (reverse){
      m_intake.reverseIntake();
    } else {
      m_intake.runIntake(); 
    }

  }
  
    /**
   * Called once the intake command ends or is interrupted.
   * 
   * @param interrupted UNKNOWN PURPOSE OF METHOD (PLEASE FILL IN DOCUMENTATION)
   * @author ziwei8658
   */
  @Override
  public void end(boolean interrupted) {
      m_intake.stopIntake();
  }

    /**
   * Returns true when the command should end. Does not end on its own, the command is ended when external schedules are false(see robot container)
   * @author ziwei8658
   */
  //
  @Override
  public boolean isFinished() {
    return false;
  }

}//end of class
