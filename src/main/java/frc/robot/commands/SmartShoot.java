
//imports
package frc.robot.commands;
import frc.robot.subsystems.ShootSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.Timer;

/**
 * Class that is used to command the robot to shoot.
 * 
 * @author ziwei8658
 */
public class SmartShoot extends Command {

  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})  //remove later

  //instance variables
  private final ShootSubsystem m_shoot;
  private final IndexerSubsystem m_indexer;
  private DriveSubsystem m_drive;
  private final Timer timer = new Timer();
  /*
   * regression shoot
   */
  public SmartShoot(ShootSubsystem m_shoot, IndexerSubsystem m_indexer) {
    this.m_shoot = m_shoot;
    this.m_indexer = m_indexer;
    addRequirements(m_shoot,m_indexer); 
  }
  
  /*
   * Discrete range shoot
   */

  @Override   
  public void initialize() {
    timer.reset();
    timer.start();
    System.out.println("SmartShoot initialized");
    
  }

  @Override
  public void execute() {
    //calculate shooter rpm
    double distance = m_drive.getRange();
    double rpm = m_shoot.getShooterRPM(distance);
    
    //end current cycle if no tag detected
    if(distance==-1)
    {
        timer.reset();
        return;
    }

    m_shoot.smartShoot(rpm);
    //use timer temporarily, change to rpm setpoint-based when tuned
    if(timer.get()>=2){
      m_indexer.runIndexVert();
      } 
  }
  /*alternative:
        if (m_shoot.atSetpoint()) {
        m_indexer.runIndexVert();
        }

        //put this in shootsubsystem
        public boolean atSetpoint()
        {
        return Math.abs(getCurrentRPM() - targetRPM) < 100;
        }
   */
  
  @Override  // Called once the command ends or is interrupted.
  public void end(boolean interrupted) {
    m_shoot.stopShooter();
    timer.stop();
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
