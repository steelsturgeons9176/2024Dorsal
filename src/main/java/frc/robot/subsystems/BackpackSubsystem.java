package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.NeoMotorConstants;

public class BackpackSubsystem extends SubsystemBase {
    private final CANSparkMax m_backpack = new CANSparkMax(NeoMotorConstants.kBackPackDeviceId, MotorType.kBrushless);

    public BackpackSubsystem() {
        m_backpack.setInverted(false);
        m_backpack.setIdleMode(IdleMode.kBrake);
        m_backpack.setSmartCurrentLimit(ArmConstants.ARM_CURRENT_LIMIT_A);

    }

    public void runBackpack(double speed) {
        m_backpack.set(speed);
    }
}
