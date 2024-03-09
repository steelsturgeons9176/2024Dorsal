package frc.robot.commands.indexer;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IndexerSubsystem;

public class RunIndexerAmp extends Command {
    private IndexerSubsystem m_indexer;

    public RunIndexerAmp(IndexerSubsystem indexer) {
        m_indexer = indexer;

        addRequirements(m_indexer);
    }

    @Override
    public void execute() {
        m_indexer.RunIndexer(-1);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean isInterrupted) {
        m_indexer.RunIndexer(0);

    }
}