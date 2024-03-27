package frc.robot.commands.vision;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.VisionSubSystem;

public class aimTele extends Command {
    
    private DriveSubsystem m_DriveSubsystem;
    private VisionSubSystem m_vision;

    public aimTele(DriveSubsystem drive, VisionSubSystem vision) {
        m_DriveSubsystem = drive;
        m_vision = vision;

        addRequirements(m_DriveSubsystem);
        addRequirements(vision);
    }

    @Override
    public void execute() {
        //m_shooter.runShooter(1);
        m_DriveSubsystem.drive(0, 0, m_vision.limelight_aim_proportional(), true, true);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean isInterrupted) {
        //m_shooter.runShooter(0);

    }
}
