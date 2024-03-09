package frc.robot.commands.manipCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class intakeFromFloor extends Command {
    IntakeSubsystem m_intake;
    FeederSubsystem m_feeder;
    IndexerSubsystem m_indexer;
    
    public intakeFromFloor(IntakeSubsystem intake, FeederSubsystem feeder, IndexerSubsystem indexer)
    {
        m_intake = intake;
        m_feeder = feeder;
        m_indexer = indexer;

        addRequirements(intake);
        addRequirements(feeder);
        addRequirements(indexer);
    }

     @Override
    public void initialize() {
    }


    @Override
    public void execute() {
        m_intake.runIntake(1);
        m_feeder.runFeeder(.8);
        m_indexer.RunIndexer(.1);
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
        m_intake.runIntake(0);
    }
}
