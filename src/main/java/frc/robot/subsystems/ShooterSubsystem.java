package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.KrakenMotorConstants;

public class ShooterSubsystem extends SubsystemBase {
    private final TalonFX m_shooterleft = new TalonFX(KrakenMotorConstants.kShooterLeftDeviceId);
    private final TalonFX m_shooterright = new TalonFX(KrakenMotorConstants.kShooterRightDeviceId);

    public ShooterSubsystem() {
        m_shooterleft.setInverted(true);
        m_shooterleft.setNeutralMode(NeutralModeValue.Coast);
        m_shooterleft.setVoltage(12);

        m_shooterright.setInverted(false);
        m_shooterright.setNeutralMode(NeutralModeValue.Coast);
        m_shooterright.setVoltage(12);
    }

    public void runShooter(double speed) {
        m_shooterleft.set(speed);
        m_shooterright.set(speed);
    }

}