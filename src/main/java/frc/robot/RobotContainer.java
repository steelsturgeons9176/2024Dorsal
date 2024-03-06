// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.arm.ArmToPosition;
import frc.robot.commands.backpack.RunBackpack;
import frc.robot.commands.climb.RunClimb;
import frc.robot.commands.descend.RunDescend;
import frc.robot.commands.feeder.RunFeeder;
import frc.robot.commands.indexer.RunIndexer;
import frc.robot.commands.intake.RunIntake;
import frc.robot.commands.shooter.RunShooter;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.BackpackSubsystem;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;

import com.pathplanner.lib.commands.PathPlannerAuto;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  public final ArmSubsystem m_arm = new ArmSubsystem();
  public final BackpackSubsystem m_backpack = new BackpackSubsystem();
  public final ClimbSubsystem m_climb = new ClimbSubsystem();
  public final FeederSubsystem m_feeder = new FeederSubsystem();
  public final ShooterSubsystem m_shooter = new ShooterSubsystem();
  public final IndexerSubsystem m_indexer = new IndexerSubsystem();
  public final IntakeSubsystem m_intake = new IntakeSubsystem();
  public final Joystick m_manipController = new Joystick(1);

  // The driver's controller
  private final CommandXboxController m_driverController = new CommandXboxController(0);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    // Configure default commands
    /*
     * m_robotDrive.setDefaultCommand(
     * // The left stick controls translation of the robot.
     * // Turning is controlled by the X axis of the right stick.
     * new RunCommand(
     * () -> m_robotDrive.drive(
     * -MathUtil.applyDeadband(m_driverController.getLeftY(),
     * OIConstants.kDriveDeadband),
     * -MathUtil.applyDeadband(m_driverController.getLeftX(),
     * OIConstants.kDriveDeadband),
     * -MathUtil.applyDeadband(m_driverController.getRightX(),
     * OIConstants.kDriveDeadband),
     * true, true),
     * m_robotDrive));
     */
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return new PathPlannerAuto("New Auto");
  }

  public final void shootNote() {
    new RunShooter(m_shooter);
    new RunIndexer(m_indexer);
  }

  public final void intakeNode() {
    new ArmToPosition(m_arm, ArmSubsystem.armPositions.INTAKE);

    new RunIntake(m_intake);
    new RunIndexer(m_indexer);
    new RunFeeder(m_feeder);
  }

  public final void ampBackpack() {
    new ArmToPosition(m_arm, ArmSubsystem.armPositions.AMP);

    new RunFeeder(m_feeder);
    new RunBackpack(m_backpack);
  }

  public final void climb() {
    new RunClimb(m_climb);
  }

  public final void descend() {
    new RunDescend(m_climb);
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
   * subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
   * passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {
    new POVButton(m_manipController, 180).whileTrue(new ArmToPosition(m_arm, ArmSubsystem.armPositions.SHOOTING));
    new POVButton(m_manipController, 270).whileTrue(new ArmToPosition(m_arm, ArmSubsystem.armPositions.AMP));
    new POVButton(m_manipController, 0).whileTrue(new ArmToPosition(m_arm, ArmSubsystem.armPositions.STOWED));
    new POVButton(m_manipController, 90).whileTrue(new ArmToPosition(m_arm, ArmSubsystem.armPositions.SOURCE));
    new JoystickButton(m_manipController, 6).whileTrue(new RunFeeder(m_feeder));
  }
}
