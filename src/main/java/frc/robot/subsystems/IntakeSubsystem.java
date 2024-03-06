package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.KrakenMotorConstants;

public class IntakeSubsystem extends SubsystemBase {
    private final TalonFX m_intake = new TalonFX(KrakenMotorConstants.kIntakeDviceId);

    public IntakeSubsystem() {
        m_intake.setInverted(false);
        m_intake.setNeutralMode(NeutralModeValue.Coast);
        m_intake.setVoltage(12);
    }

    public void runIntake(double speed) {
        m_intake.set(speed);
    }
}
