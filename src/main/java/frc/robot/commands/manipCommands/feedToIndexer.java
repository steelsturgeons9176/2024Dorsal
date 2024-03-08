package frc.robot.commands.manipCommands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.feeder.ReverseFeeder;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class feedToIndexer extends Command {

    private FeederSubsystem m_feeder;

    double m_time = 0;

    public feedToIndexer(FeederSubsystem feeder) {
        m_feeder = feeder;

        addRequirements(m_feeder);
    }
    
    @Override
    public void initialize() {
        m_time = Timer.getFPGATimestamp();
    }

    @Override
    public void execute() {
        new ReverseFeeder(m_feeder);
    }

    @Override
    public boolean isFinished() {
        if(m_time + .1 >= Timer.getFPGATimestamp())
        {
            return true; 
        }
        return false;
    }

    @Override
    public void end(boolean isInterrupted) {
        m_feeder.runFeeder(0);
    }


}
