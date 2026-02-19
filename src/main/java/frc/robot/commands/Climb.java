// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants.ClimberConstants;
import frc.robot.subsystems.ClimberSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

//  An example command that uses an example subsystem. 
public class Climb extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final ClimberSubsystem m_climber;
  private boolean toggle; //false = climber extended, true = climber retracted - carter

  
//    * Creates a new ExampleCommand.
//    *
//    * @param subsystem The subsystem used by this command.
//    *
  public Climb(ClimberSubsystem subsystem, boolean toggle) {
    this.m_climber = subsystem;
    this.toggle = toggle;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("climb start");
    // if (m_climber.m_climbAbsoluteEncoder.getPosition()<= ClimberConstants.kclimberLowerThreshhold){
    //     toggle = false;
    // } else {
    //     toggle = true;
    // }
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if (toggle == true){
        m_climber.ClimberUp();
    } else {
        m_climber.ClimberDown();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_climber.stopClimber();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (toggle){
      return m_climber.m_climbAbsoluteEncoder.getPosition() >= ClimberConstants.kclimberUpperThreshhold;
    } else {
      return m_climber.m_climbAbsoluteEncoder.getPosition() <= ClimberConstants.kclimberLowerThreshhold;
    }
  }
}
