package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

public class AlignToHub extends Command {

    private final DriveSubsystem driveSubsystem;

    public AlignToHub(DriveSubsystem driveSubsystem) {
        this.driveSubsystem = driveSubsystem;

        addRequirements(driveSubsystem);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        driveSubsystem.pointAtHub();
    }

    @Override
    public void end(boolean interrupted) {
        driveSubsystem.drive(0, 0, 0, false);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
