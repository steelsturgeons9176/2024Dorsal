package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.KrakenMotorConstants;

public class IndexerSubsystem extends SubsystemBase {
    private final TalonFX m_indexer = new TalonFX(KrakenMotorConstants.kIdexerDeviceID);

    public IndexerSubsystem() {
        m_indexer.setInverted(false);
        m_indexer.setNeutralMode(NeutralModeValue.Coast);
        m_indexer.setVoltage(12);
    }

    public void runIntake(double speed) {
        m_indexer.set(speed);
    }
}
