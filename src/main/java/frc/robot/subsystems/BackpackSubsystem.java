package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.KrakenMotorConstants;

public class BackpackSubsystem extends SubsystemBase {
    private final TalonFX m_backpack = new TalonFX(KrakenMotorConstants.kAmpBackpackID);

    public BackpackSubsystem() {
       m_backpack.setInverted(false);
       m_backpack.setNeutralMode(NeutralModeValue.Coast);
        m_backpack.setVoltage(12);
        m_backpack.set(0);

    }

    public void runBackpack(double speed) {
      m_backpack.set(speed);
    }
}
