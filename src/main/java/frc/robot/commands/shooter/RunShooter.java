package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

public class RunShooter extends Command {
    private ShooterSubsystem m_shooter;

    public RunShooter(ShooterSubsystem shooter) {
        m_shooter = shooter;

        addRequirements(m_shooter);
    }

    @Override
    public void execute() {
        m_shooter.spinShootNote(.7,.4); //.9, .8 
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