package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.NeoMotorConstants;

public class ClimbSubsystem extends SubsystemBase {
    private final CANSparkMax m_climb = new CANSparkMax(NeoMotorConstants.kClimbDeviceId, MotorType.kBrushless);

    public ClimbSubsystem() {
        m_climb.setInverted(false);
        m_climb.setIdleMode(IdleMode.kBrake);
        m_climb.setSmartCurrentLimit(ArmConstants.ARM_CURRENT_LIMIT_A);
    }

    public void runClimb(double speed) {
        m_climb.set(speed);
    }

    public void runDescend(double speed) {
        m_climb.set(-speed);
    }
}
