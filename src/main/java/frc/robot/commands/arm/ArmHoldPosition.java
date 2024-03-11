package frc.robot.commands.arm;

import frc.robot.subsystems.ArmSubsystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

public class ArmHoldPosition extends Command {
    private ArmSubsystem m_arm;
    private ArmSubsystem.armPositions m_targetPos;
    private boolean m_keepRunning;
    private boolean reachedTarget = false;
    //private double m_startTime = 0;

    public ArmHoldPosition(ArmSubsystem arm, ArmSubsystem.armPositions holdPos){
        m_arm = arm;
        m_targetPos = holdPos;
        m_keepRunning = false;
        addRequirements(m_arm);
    }

    public ArmHoldPosition(ArmSubsystem arm, ArmSubsystem.armPositions holdPos, boolean keepRunning){
        m_arm = arm;
        m_targetPos = holdPos;
        m_keepRunning = keepRunning;
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
       m_arm.holdArm(m_targetPos);
    }

    @Override
    public boolean isFinished(){
        return false;
    }

    @Override
    public void end(boolean isInterrupted){
        if (!m_keepRunning) m_arm.noArmPower();
    }
}