package frc.robot.commands.feeder;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.FeederSubsystem;

public class RunFeeder extends Command {
    private FeederSubsystem m_feeder;

    public RunFeeder(FeederSubsystem feeder) {
        m_feeder = feeder;

        addRequirements(m_feeder);
    }

    @Override
    public void execute() {
        m_feeder.runFeeder(1);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean isInterrupted) {
        m_feeder.runFeeder(0);
    }
}