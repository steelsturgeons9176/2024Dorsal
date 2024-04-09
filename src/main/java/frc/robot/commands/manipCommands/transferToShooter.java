package frc.robot.commands.manipCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class transferToShooter extends Command {
    FeederSubsystem m_feeder;
    IndexerSubsystem m_indexer;
    
    public transferToShooter(FeederSubsystem feeder, IndexerSubsystem indexer)
    {
        m_feeder = feeder;
        m_indexer = indexer;

        addRequirements(feeder);
        addRequirements(indexer);
    }

     @Override
    public void initialize() {
    }


    @Override
    public void execute() {
        m_feeder.runFeeder(.05);  // tested values:.15 .1
        m_indexer.RunIndexer(.15); // tested values: .3 .2
    }

    @Override
    public boolean isFinished() {
        if(m_indexer.DetectColor())
        {
            return true; 
        }
        return false;
    }

    @Override
    public void end(boolean isInterrupted) {
        m_feeder.runFeeder(0);
        m_indexer.RunIndexer(0);
    }
}
