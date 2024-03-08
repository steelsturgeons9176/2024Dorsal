package frc.robot.subsystems;

import java.util.EnumMap;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder.Type;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.NeoMotorConstants;

public class ArmSubsystem extends SubsystemBase {
    public enum armPositions{
        STOWED, 
        AMP, 
        INTAKE,
        SOURCE,
        SUBSHOT,
        PODSHOT
    }

    private final CANSparkMax m_armRight = new CANSparkMax(NeoMotorConstants.kArmRightDeviceId, MotorType.kBrushless);
    private final CANSparkMax m_armLeft = new CANSparkMax(NeoMotorConstants.kArmLeftDeviceId, MotorType.kBrushless);

    private final AbsoluteEncoder armAbsEncoder = m_armRight.getAbsoluteEncoder(Type.kDutyCycle);

    private final PIDController m_AbsPidController = new PIDController(0.0, 0.0, 0.0);

    EnumMap<armPositions, Double> mapAbs = new EnumMap<>(armPositions.class);

    double m_speed = 0.0;


    public ArmSubsystem () {

        mapAbs.put(armPositions.STOWED, ArmConstants.STOWED);
        mapAbs.put(armPositions.AMP, ArmConstants.AMP);
        mapAbs.put(armPositions.SOURCE, ArmConstants.SOURCE);
        mapAbs.put(armPositions.SUBSHOT, ArmConstants.SUBSHOT);
        mapAbs.put(armPositions.PODSHOT, ArmConstants.PODSHOT);
        mapAbs.put(armPositions.INTAKE, ArmConstants.INTAKE);

        m_armRight.setInverted(true);
        m_armRight.setIdleMode(IdleMode.kBrake);
        m_armRight.setSmartCurrentLimit(ArmConstants.ARM_CURRENT_LIMIT_A);
        //m_armLeft.setInverted(true);
        m_armLeft.setIdleMode(IdleMode.kBrake);
        m_armLeft.setSmartCurrentLimit(ArmConstants.ARM_CURRENT_LIMIT_A);
        m_armLeft.follow(m_armRight, true);

        m_AbsPidController.enableContinuousInput(0, 1);


        m_armRight.burnFlash();
        m_armLeft.burnFlash();


    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Arm Relative Enc", m_armRight.getEncoder().getPosition());
        SmartDashboard.putNumber("ArmABS Absolute", armAbsEncoder.getPosition());
        SmartDashboard.putNumber("Arm oCurrent", m_armRight.getOutputCurrent());
        SmartDashboard.putNumber("Arm Motor Speed", m_speed);
        SmartDashboard.putNumber("LeftMotor", m_armRight.getOutputCurrent());
        SmartDashboard.putNumber("Right Motor", m_armLeft.getOutputCurrent());

        // SmartDashboard.putNumber("ArmABS Offset", armAbsEncoder.getPositionOffset());  
        //   Might use global that is set by drive periodic to indicate if driving too fast.

        m_speed = m_armRight.getEncoder().getVelocity();

        if (((armAbsEncoder.getPosition() < ArmConstants.kMinHeightAbs) && (m_speed < 0)) ||
            ((armAbsEncoder.getPosition() > ArmConstants.kMaxHeightAbs) && (m_speed > 0))) {
                m_armRight.set(0);
            }

    }

    public boolean raiseArmAbs(armPositions position){
        if (((armAbsEncoder.getPosition() < ArmConstants.kMinHeightAbs) && (position == armPositions.STOWED)) ||
            ((armAbsEncoder.getPosition() > ArmConstants.kMaxHeightAbs) && (position == armPositions.AMP))) {
            m_armRight.set(0);
            return true;
        }

        // For LVLTRE, LVLTWO, and HOME
        switch (position) {
            case AMP:
                m_AbsPidController.setP(2.7);
                m_AbsPidController.setI(.2);
                m_AbsPidController.setD(0);
            case SUBSHOT:
                m_AbsPidController.setP(2.7);
                m_AbsPidController.setI(.5);
                m_AbsPidController.setD(0);
            case STOWED:
                m_AbsPidController.setP(1.3);
                m_AbsPidController.setI(.2);
                m_AbsPidController.setD(0);
                break;
        // For LVLONE and CONESTOW
            case SOURCE:
                m_AbsPidController.setP(2.7);
                m_AbsPidController.setI(.2);
                m_AbsPidController.setD(0);
            default:
                m_AbsPidController.setP(2.7);
                m_AbsPidController.setI(.2);
                m_AbsPidController.setD(0);
                break;
        }

        double ref = mapAbs.get(position);

        double pidOut = MathUtil.clamp(
            m_AbsPidController.calculate(armAbsEncoder.getPosition(),ref),
            Constants.ArmConstants.kArmMinOutput, Constants.ArmConstants.kArmMaxOutput);
            
        SmartDashboard.putNumber("Arm Abs Target Pos", ref);
        m_armRight.set(pidOut);
        if(atPosition(position))
        {
            return true;
        }
        return false;
    }

    public void holdArm(armPositions position)
    {
        if (((armAbsEncoder.getPosition() < ArmConstants.kMinHeightAbs) && (position == armPositions.STOWED)) ||
            ((armAbsEncoder.getPosition() > ArmConstants.kMaxHeightAbs) && (position == armPositions.AMP))) {
            m_armRight.set(0);
            return;
        }

        // For LVLTRE, LVLTWO, and HOME
        switch (position) {
            case AMP:
                m_AbsPidController.setP(2);
                m_AbsPidController.setI(.5);
                m_AbsPidController.setD(.1);
            case SUBSHOT:
                m_AbsPidController.setP(2);
                m_AbsPidController.setI(.5);
                m_AbsPidController.setD(.1);
            case STOWED:
                m_AbsPidController.setP(1);
                m_AbsPidController.setI(.5);
                m_AbsPidController.setD(.1);
                break;
        // For LVLONE and CONESTOW
            case SOURCE:
                m_AbsPidController.setP(2);
                m_AbsPidController.setI(.5);
                m_AbsPidController.setD(.1);
            default:
                m_AbsPidController.setP(2);
                m_AbsPidController.setI(.5);
                m_AbsPidController.setD(.1);
                break;
        }

        double ref = mapAbs.get(position);

        double pidOut = MathUtil.clamp(
            m_AbsPidController.calculate(armAbsEncoder.getPosition(),ref),
            Constants.ArmConstants.kArmMinOutput, Constants.ArmConstants.kArmMaxOutput);
        m_armRight.set(pidOut);
    }

    public boolean atPosition(armPositions pos){
        double currentEncoderPosition = armAbsEncoder.getPosition();
        return (Math.abs(currentEncoderPosition - mapAbs.get(pos)) < Constants.ArmConstants.kAllowedErrAbs);
    }

    public void noArmPower()
    {
        m_armRight.set(0);
    }
}
