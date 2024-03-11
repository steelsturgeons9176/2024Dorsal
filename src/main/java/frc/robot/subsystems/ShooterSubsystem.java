package frc.robot.subsystems;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.KrakenMotorConstants;
import frc.utils.LinearProfile;
import lombok.RequiredArgsConstructor;
import lombok.Setter;

public class ShooterSubsystem extends SubsystemBase {
    private final TalonFX m_shooterLeft = new TalonFX(KrakenMotorConstants.kShooterLeftDeviceId);
    private final TalonFX m_shooterRight = new TalonFX(KrakenMotorConstants.kShooterRightDeviceId);

  private final LinearProfile leftProfile;
  private final LinearProfile rightProfile;
  private SimpleMotorFeedforward ff = new SimpleMotorFeedforward(Constants.ShooterConstants.kS, Constants.ShooterConstants.kV, 0);

  private boolean wasClosedLoop = false;
  private boolean closedLoop = false;
  @Setter private BooleanSupplier prepareShootSupplier = () -> false;

private final VelocityTorqueCurrentFOC velocityControl =
    new VelocityTorqueCurrentFOC(0.0).withUpdateFreqHz(0.0);

    private final NeutralOut neutralControl = new NeutralOut().withUpdateFreqHz(0.0);

  @RequiredArgsConstructor
  public enum Goal {
    STOP(() -> 0, () -> 0),
    IDLE(() -> Constants.ShooterConstants.idleLeftRpm, () -> Constants.ShooterConstants.idleRightRpm),
    SHOOT(() -> Constants.ShooterConstants.shootingLeftRpm,() ->  Constants.ShooterConstants.shootingRightRpm),
    INTAKE(() -> Constants.ShooterConstants.intakingRpm,() ->  Constants.ShooterConstants.intakingRpm),
    EJECT(() -> Constants.ShooterConstants.ejectingRpm,() ->  Constants.ShooterConstants.ejectingRpm),
    CHARACTERIZING(() -> 0.0, () -> 0.0);

    private final DoubleSupplier leftGoal;
    private final DoubleSupplier rightGoal;

    private double getLeftGoal() {
      return leftGoal.getAsDouble();
    }

    private double getRightGoal() {
      return rightGoal.getAsDouble();
    }
  }

  public enum IdleMode {
    TELEOP,
    AUTO
  }

  private boolean isDrawingHighCurrent() {
    return Math.abs(m_shooterLeft.getSupplyCurrent().getValueAsDouble()) > 50.0
        || Math.abs(m_shooterRight.getSupplyCurrent().getValueAsDouble()) > 50.0;
  }

    private Goal goal = Goal.IDLE;

    public ShooterSubsystem() {
        m_shooterLeft.setInverted(true);
        m_shooterLeft.setNeutralMode(NeutralModeValue.Coast);
        m_shooterLeft.setVoltage(12);
        m_shooterLeft.set(0);

        m_shooterRight.setInverted(false);
        m_shooterRight.setNeutralMode(NeutralModeValue.Coast);
        m_shooterRight.setVoltage(12);
        m_shooterRight.set(0);

        leftProfile = new LinearProfile(Constants.ShooterConstants.maxAcclerationRpmPerSec, .02);
        rightProfile = new LinearProfile(Constants.ShooterConstants.maxAcclerationRpmPerSec, .02);

        setDefaultCommand(runOnce(() -> setGoal(Goal.IDLE)).withName("Flywheels Idle"));
    }

    @Override
    public void periodic()
    {
        if (DriverStation.isDisabled()) {
            setGoal(Goal.STOP);
        }
        if (!closedLoop && wasClosedLoop) {
            leftProfile.reset();
            rightProfile.reset();
            wasClosedLoop = false;
        }

            // Get goal
        double leftGoal = goal.getLeftGoal();
        double rightGoal = goal.getRightGoal();
        boolean idlePrepareShoot = goal == Goal.IDLE && prepareShootSupplier.getAsBoolean();

        if (idlePrepareShoot) {
            leftGoal = Goal.SHOOT.getLeftGoal() * Constants.ShooterConstants.prepareShootMultiplier;
            rightGoal = Goal.SHOOT.getRightGoal() * Constants.ShooterConstants.prepareShootMultiplier;
          }

        if (closedLoop) {
            // Update goals
            leftProfile.setGoal(goal.getLeftGoal());
            rightProfile.setGoal(goal.getRightGoal());
            m_shooterLeft.setControl(velocityControl.withVelocity(leftProfile.calculateSetpoint() / 60.0));
            m_shooterRight.setControl(velocityControl.withVelocity(rightProfile.calculateSetpoint() / 60.0));
          }

          if (closedLoop || idlePrepareShoot) {
                // Update goals
                leftProfile.setGoal(leftGoal);
                rightProfile.setGoal(rightGoal);
                double leftSetpoint = leftProfile.calculateSetpoint();
                double rightSetpoint = rightProfile.calculateSetpoint();
                m_shooterLeft.setControl(
                    velocityControl.withVelocity(leftSetpoint / 60.0).withFeedForward(ff.calculate(leftSetpoint)));
                m_shooterRight.setControl(
                    velocityControl.withVelocity(rightSetpoint / 60.0).withFeedForward(ff.calculate(rightSetpoint)));
                //RobotState.getInstance().setFlywheelAccelerating(!atGoal() || isDrawingHighCurrent());
                } 
            else if (goal == Goal.IDLE) {
                //RobotState.getInstance().setFlywheelAccelerating(false);
                m_shooterLeft.setControl(neutralControl);
                m_shooterRight.setControl(neutralControl);
            }

    }

    public void runShooter(double speed) {
        m_shooterLeft.set(speed);
        m_shooterRight.set(speed);
    }

    private void setGoal(Goal goal) {
        if (goal == Goal.CHARACTERIZING || goal == Goal.STOP) {
            wasClosedLoop = closedLoop;
            closedLoop = false;
            return; // Don't set a goal
        }
        // If not already controlling to requested goal
        // set closed loop false
        closedLoop = this.goal == goal;
        // Enable close loop
        if (!closedLoop) {
            leftProfile.setGoal(goal.getLeftGoal(), m_shooterLeft.getRotorVelocity().getValueAsDouble());
            rightProfile.setGoal(goal.getRightGoal(), m_shooterLeft.getRotorVelocity().getValueAsDouble());
            closedLoop = true;
        }
        this.goal = goal;
    }

      public boolean atGoal() {
        return goal == Goal.IDLE
            || (leftProfile.getCurrentSetpoint() == goal.getLeftGoal()
                && rightProfile.getCurrentSetpoint() == goal.getRightGoal());
      }

    public Command shootCommand() {
        return startEnd(() -> setGoal(Goal.SHOOT), () -> setGoal(Goal.IDLE))
            .withName("Flywheels Shoot");
      }
    
      public Command intakeCommand() {
        return startEnd(() -> setGoal(Goal.INTAKE), () -> setGoal(Goal.IDLE))
            .withName("Flywheels Intake");
      }

}