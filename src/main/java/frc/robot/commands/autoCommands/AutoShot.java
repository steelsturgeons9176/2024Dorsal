package frc.robot.commands.autoCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class AutoShot extends Command {
    FeederSubsystem m_feeder;
    IndexerSubsystem m_indexer;
    ShooterSubsystem m_shooter;

    double m_startTime = 0;

    public AutoShot(FeederSubsystem feeder, IndexerSubsystem indexer, ShooterSubsystem shooter)
    {
        m_feeder = feeder;
        m_indexer = indexer;
        m_shooter = shooter;

        addRequirements(feeder);
        addRequirements(indexer);
        addRequirements(shooter);
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
        m_feeder.runFeeder(1);
        m_indexer.RunIndexer(1);
        m_shooter.runShooter(1);
    }

    @Override
    public boolean isFinished(){
        if(getTime() >= .25f)
        {
            return true; 
        }
        return false;
    }

    @Override
    public void end(boolean isInterrupted){
        m_feeder.runFeeder(1);
        m_indexer.RunIndexer(1);
        m_shooter.runShooter(1);
    }

}
