package frc.robot.commands.autoCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ArmPositionAuto  extends Command{
    private ArmSubsystem m_arm;
    private ArmSubsystem.armPositions m_targetPos;
    private ShooterSubsystem m_shooter;
    private boolean m_keepRunning;
    private boolean reachedTarget = false;
    //private double m_startTime = 0;

    public ArmPositionAuto(ArmSubsystem arm, ArmSubsystem.armPositions pos, ShooterSubsystem shooter){
        m_arm = arm;
        m_shooter = shooter;
        m_targetPos = pos;
        m_keepRunning = false;
        addRequirements(m_arm);
    }

    @Override
    public void initialize() {
      //m_startTime = Timer.getFPGATimestamp();
    }
    /* 
    public double getTime() {
        return Timer.getFPGATimestamp() - m_startTime;
    }*/

    @Override
    public void execute(){
       reachedTarget = m_arm.raiseArmAbs(m_targetPos);
       m_shooter.runShooter(1);
    }

    @Override
    public boolean isFinished(){
        if(reachedTarget)
        {
            return true;
        }
        return false;
    }

    @Override
    public void end(boolean isInterrupted){
        if (!m_keepRunning) m_arm.raiseArmAbs(m_targetPos);
    }
}

