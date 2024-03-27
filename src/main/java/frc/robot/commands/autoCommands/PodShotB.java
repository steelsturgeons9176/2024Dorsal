package frc.robot.commands.autoCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.arm.ArmToPosition;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.ArmSubsystem.armPositions;

public class PodShotB extends Command {
    ArmSubsystem m_arm;
    IndexerSubsystem m_indexer;
    ShooterSubsystem m_shooter;

    double m_startTime = 0;
    boolean shooting = false;
    boolean noteExitedIndexer = false;
    boolean noteShotAtSpeaker = false;

    public PodShotB(ArmSubsystem arm, IndexerSubsystem indexer, ShooterSubsystem shooter)
    {
        m_arm = arm;
        m_indexer = indexer;
        m_shooter = shooter;

        addRequirements(arm);
        addRequirements(indexer);
        addRequirements(shooter);
    }

    @Override
    public void initialize() {
      m_startTime = Timer.getFPGATimestamp();
      m_arm.raiseArmAbs(armPositions.PODSHOT);
      m_shooter.runShooter(1);
        shooting = false;
        noteExitedIndexer = false;
        noteShotAtSpeaker = false;
        
    }

    public double getTime() {
        return Timer.getFPGATimestamp() - m_startTime;
    }

    @Override
    public void execute(){
        m_shooter.runShooter(1);
        if(getTime() >= 1.0f && shooting == false)
        {
            m_indexer.RunIndexer(1);
            shooting = true;
        }
        if(shooting && noteExitedIndexer == false)
        {
            if(!m_indexer.DetectColor())
            {
                noteExitedIndexer = true;
                m_startTime = Timer.getFPGATimestamp();
            }
        }
        if(noteExitedIndexer && getTime() >= .1f)
        {
            noteShotAtSpeaker = true; 
        }
    }

    @Override
    public boolean isFinished(){
        return noteShotAtSpeaker ? true:false;
    }

    @Override
    public void end(boolean isInterrupted){
        m_indexer.RunIndexer(0);
        m_shooter.runShooter(0);
    }

}
