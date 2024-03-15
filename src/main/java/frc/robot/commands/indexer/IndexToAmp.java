package frc.robot.commands.indexer;

import org.ejml.equation.IntegerSequence.Combined;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.IndexerSubsystem;

public class IndexToAmp extends Command {
    FeederSubsystem m_feeder;
    IndexerSubsystem m_indexer;
    double m_startTime = 0;
    public IndexToAmp(FeederSubsystem feeder, IndexerSubsystem indexer)
    {
        m_feeder = feeder;
        m_indexer = indexer;
    }

    @Override
    public void initialize()
    {
        //m_startTime = Timer.getFPGATimestamp();
    }

    public double getTime() {
        return Timer.getFPGATimestamp() - m_startTime;
    }

    @Override
    public void execute() {
        m_indexer.RunIndexer(-.4);
        m_feeder.runFeeder(.4);
        if(!m_indexer.DetectColor())
        {
            m_startTime = Timer.getFPGATimestamp();
        }
    }

    @Override
    public boolean isFinished() {
        if(getTime() >= .2f)
        {
            return true;
        }
        return false;
    }

    @Override
    public void end(boolean isInterrupted) {
        m_indexer.RunIndexer(0);
        m_feeder.runFeeder(0);
    }
}
