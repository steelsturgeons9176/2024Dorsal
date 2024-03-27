package frc.robot.commands.backpack;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.BackpackSubsystem;
import frc.robot.subsystems.FeederSubsystem;

public class RunBackpack extends Command {
    
    private BackpackSubsystem m_backpack;
    private FeederSubsystem m_feeder;

    public RunBackpack(BackpackSubsystem backpack, FeederSubsystem feeder) {
        m_backpack = backpack;
        m_feeder = feeder;

        addRequirements(m_backpack);
        addRequirements(feeder);
    }

    @Override
    public void execute() {
        m_feeder.runFeeder(-1);
        m_backpack.runBackpack(-1);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean isInterrupted) {
        m_backpack.runBackpack(0);
        m_feeder.runFeeder(0);
    }
}
