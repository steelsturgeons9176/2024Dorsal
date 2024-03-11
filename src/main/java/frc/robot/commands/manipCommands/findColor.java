package frc.robot.commands.manipCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IndexerSubsystem;

public class findColor extends Command {
    IndexerSubsystem m_indexer;
    public findColor(IndexerSubsystem indexer)
    {
        m_indexer = indexer;
    }

    @Override
    public void execute() {
        m_indexer.DetectColor();
    }
}
