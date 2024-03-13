package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

public class RunShooterReverse extends Command {
    private ShooterSubsystem m_shooter;

    public RunShooterReverse(ShooterSubsystem shooter) {
        m_shooter = shooter;

        addRequirements(m_shooter);
    }

    @Override
    public void execute() {
        m_shooter.runShooter(-.4);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean isInterrupted) {
        m_shooter.runShooter(0);

    }
}