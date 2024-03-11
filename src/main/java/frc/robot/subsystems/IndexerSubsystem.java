package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.KrakenMotorConstants;

public class IndexerSubsystem extends SubsystemBase {
    private final TalonFX m_indexer = new TalonFX(KrakenMotorConstants.kIdexerDeviceID);
    
    private boolean noteDetected = false;
    private final I2C.Port i2cPort = I2C.Port.kOnboard;
    private final ColorSensorV3 m_colorSensor = new ColorSensorV3(i2cPort);
    private final ColorMatch m_colorMatcher = new ColorMatch();

    private final Color kOrangeTarget = new Color(0.532715, 0.371582, 0.095947);
    private final Color kGreenTarget = new Color(0.367188, 0.455811,0.177246);

    public IndexerSubsystem() {
        m_indexer.setInverted(false);
        m_indexer.setNeutralMode(NeutralModeValue.Coast);
        m_indexer.setVoltage(12);
        m_indexer.set(0);

        m_colorMatcher.addColorMatch(kOrangeTarget);
        m_colorMatcher.addColorMatch(kGreenTarget);
    }

    public void RunIndexer(double speed) {
        m_indexer.set(speed);
    }

    public boolean DetectColor()
    {
        Color detectedColor = m_colorSensor.getColor();
        ColorMatchResult match = m_colorMatcher.matchClosestColor(detectedColor);
        double IR = m_colorSensor.getIR();

        /**
         * Open Smart Dashboard or Shuffleboard to see the color detected by the 
         * sensor.
         */
        SmartDashboard.putNumber("Red", detectedColor.red);
        SmartDashboard.putNumber("Green", detectedColor.green);
        SmartDashboard.putNumber("Blue", detectedColor.blue);
        SmartDashboard.putBoolean("Note Detected", noteDetected);
        if(match.color == kOrangeTarget)
        {
            noteDetected = true;
            return true;
        }
        else
        {
            noteDetected = false;
        }
        return false;
    }
}
