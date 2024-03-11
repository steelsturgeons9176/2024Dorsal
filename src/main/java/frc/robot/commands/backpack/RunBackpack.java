package frc.robot.commands.backpack;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.BackpackSubsystem;

public class RunBackpack extends Command {
    
    private BackpackSubsystem m_backpack;

    public RunBackpack(BackpackSubsystem backpack) {
        m_backpack = backpack;

        addRequirements(m_backpack);
    }

    @Override
    public void execute() {
        m_backpack.runBackpack(1);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean isInterrupted) {
        m_backpack.runBackpack(0);
    }
}
