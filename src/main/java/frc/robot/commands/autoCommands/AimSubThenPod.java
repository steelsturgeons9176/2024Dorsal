package frc.robot.commands.autoCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class AimSubThenPod  extends Command{
    private ArmSubsystem m_arm;
    private ArmSubsystem.armPositions m_targetPos;
    private ShooterSubsystem m_shooter;
    private boolean m_keepRunning;
    private boolean reachedTarget = false;
    private double m_startTime = 0;

    public AimSubThenPod(ArmSubsystem arm, ArmSubsystem.armPositions pos){
        m_arm = arm;
        m_targetPos = pos;
        m_keepRunning = false;
        addRequirements(m_arm);
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
        m_arm.raiseArmAbs(m_targetPos);
        reachedTarget = m_arm.atPosition();
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
        if (!m_keepRunning) m_arm.raiseArmAbs(m_targetPos);
    }
}

