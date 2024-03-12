package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.NeoMotorConstants;

public class ClimbSubsystem extends SubsystemBase {
    private final CANSparkMax m_leftClimber = new CANSparkMax(NeoMotorConstants.kLeftClimberID, MotorType.kBrushless);
    private final CANSparkMax m_rightClimber= new CANSparkMax(NeoMotorConstants.kRightClimberID, MotorType.kBrushless);

    public ClimbSubsystem() {
        m_leftClimber.setInverted(false);
        m_leftClimber.setIdleMode(IdleMode.kBrake);
        m_leftClimber.setSmartCurrentLimit(ArmConstants.ARM_CURRENT_LIMIT_A);
        
        m_rightClimber.setInverted(true);
        m_rightClimber.setIdleMode(IdleMode.kBrake);
        m_rightClimber.setSmartCurrentLimit(ArmConstants.ARM_CURRENT_LIMIT_A);
    }

    public void runClimb(double speed) {
        m_rightClimber.set(speed);
        m_leftClimber.set(speed);
    }

    public void runDescend(double speed) {
        m_leftClimber.set(-speed);
        m_rightClimber.set(-speed);
    }
}
