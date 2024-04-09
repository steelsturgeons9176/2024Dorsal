package frc.robot.commands.manipCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.VisionSubSystem;

public class driveAimAuto  extends Command{
    private double m_startTime = 0;
    DriveSubsystem m_drive;
    VisionSubSystem m_vision;

    public driveAimAuto(DriveSubsystem drive, VisionSubSystem vison){
        m_drive = drive;
        m_vision = vison;
        addRequirements(m_drive);
    }

    @Override
    public void initialize() {
      m_startTime = Timer.getFPGATimestamp();
    }
     
    public double getTime() {
        return Timer.getFPGATimestamp() - m_startTime;
    }

    @Override
    public void execute(){
        m_drive.drive(m_vision.limelight_range_proportional(), 0, m_vision.limelight_aim_proportional(), true, true);
    }

    @Override
    public boolean isFinished(){
        if(getTime() >= 1.0f)
        {
            return true;
        }
        return false;
    }

    @Override
    public void end(boolean isInterrupted){
        m_drive.drive(0,0, 0, true, true);
    }
}

