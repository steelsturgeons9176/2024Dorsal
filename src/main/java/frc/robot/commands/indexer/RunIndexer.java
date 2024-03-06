package frc.robot.commands.indexer;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IndexerSubsystem;

public class RunIndexer extends Command {
    private IndexerSubsystem m_indexer;

    public RunIndexer(IndexerSubsystem indexer) {
        m_indexer = indexer;

        addRequirements(m_indexer);
    }

    @Override
    public void execute() {
        m_indexer.runIntake(1);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean isInterrupted) {
        m_indexer.runIntake(0);

    }
}