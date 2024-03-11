package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.KrakenMotorConstants;

public class FeederSubsystem extends SubsystemBase {
    private final TalonFX m_feeder = new TalonFX(KrakenMotorConstants.kFeederDeviceId);

    public FeederSubsystem() {
        m_feeder.setInverted(false);
        m_feeder.setNeutralMode(NeutralModeValue.Coast);
        m_feeder.setVoltage(12);
        m_feeder.set(0);

    }

    public void runFeeder(double speed) {
        m_feeder.set(speed);
    }
}
