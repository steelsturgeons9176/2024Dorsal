package frc.robot.commands.autoCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.arm.ArmToPosition;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.ArmSubsystem.armPositions;

public class StopMotors extends Command {
    ArmSubsystem m_arm;
    IndexerSubsystem m_indexer;
    ShooterSubsystem m_shooter;

    double m_startTime = 0;

    public StopMotors(IndexerSubsystem indexer, ShooterSubsystem shooter)
    {
        m_indexer = indexer;
        m_shooter = shooter;

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
        m_shooter.runShooter(1);
    }

    @Override
    public boolean isFinished(){
        if(getTime() >= .5f)
        {
            return true; 
        }
        return false;
    }

    @Override
    public void end(boolean isInterrupted){
        m_indexer.RunIndexer(0);
        m_shooter.runShooter(0);
    }

}
