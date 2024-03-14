package frc.robot.commands.climb;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimbSubsystem;

public class RunClimbLeftDown extends Command {

    private ClimbSubsystem m_climb;

    public RunClimbLeftDown(ClimbSubsystem climb) {
        m_climb = climb;

        addRequirements(m_climb);
    }

    @Override
    public void execute() {
        m_climb.runClimbLeft(-1);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean isInterrupted) {
        m_climb.runClimb(0);
    }

}
