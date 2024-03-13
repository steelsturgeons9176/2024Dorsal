package frc.robot.commands.manipCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class intakeFromSource extends Command {

    ShooterSubsystem m_shooter;
    IndexerSubsystem m_indexer;
    boolean sawNote;


    public intakeFromSource(ShooterSubsystem shooter, IndexerSubsystem indexer)
    {
        m_shooter = shooter;
        m_indexer = indexer;
        sawNote = false;

        addRequirements(indexer);
        addRequirements(shooter);
    }

    @Override
    public void initialize()
    {
        sawNote = false;
    }

    @Override
    public void execute()
    {
        m_shooter.runShooter(-.3);
        m_indexer.RunIndexer(-.1);
        if(m_indexer.DetectColor())
        {
            sawNote = true;
        }
    }

    @Override
    public boolean isFinished()
    {
        if(!m_indexer.DetectColor() && sawNote){
            return true;
        }
        return false;
    }

    @Override
    public void end(boolean isInterrupted)
    {
        m_shooter.runShooter(0);
        m_indexer.RunIndexer(0);
    }
    
}
