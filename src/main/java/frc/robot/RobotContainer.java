// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.arm.ArmToPosition;
import frc.robot.commands.backpack.RunBackpack;
import frc.robot.commands.climb.RunClimb;
import frc.robot.commands.descend.RunDescend;
import frc.robot.commands.feeder.ReverseFeeder;
import frc.robot.commands.feeder.RunFeeder;
import frc.robot.commands.indexer.RunIndexerAmp;
import frc.robot.commands.indexer.RunIndexerShooter;
import frc.robot.commands.intake.RunIntake;
import frc.robot.commands.intake.RunIntakeUnjam;
import frc.robot.commands.manipCommands.feedToIndexer;
import frc.robot.commands.manipCommands.findColor;
import frc.robot.commands.manipCommands.intakeFromFloor;
import frc.robot.commands.manipCommands.manipIntake;
import frc.robot.commands.manipCommands.stowArm;
import frc.robot.commands.shooter.RunShooter;
import frc.robot.commands.shooter.RunShooterReverse;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.BackpackSubsystem;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.ArmSubsystem.armPositions;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

import javax.naming.PartialResultException;

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
  

  // The driver's controller
  private final CommandXboxController m_driverController = new CommandXboxController(0);

  public final CommandJoystick m_manipController = new CommandJoystick(1);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    // Configure default commands
    
     m_robotDrive.setDefaultCommand(
      // The left stick controls translation of the robot.
      // Turning is controlled by the X axis of the right stick.
      new RunCommand(
      () -> m_robotDrive.drive(
      -MathUtil.applyDeadband(m_driverController.getLeftY(),
      OIConstants.kDriveDeadband),
      -MathUtil.applyDeadband(m_driverController.getLeftX(),
      OIConstants.kDriveDeadband),
      -MathUtil.applyDeadband(m_driverController.getRightX(),
      OIConstants.kDriveDeadband),
      true, true),
      m_robotDrive));
     
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
    new RunIndexerShooter(m_indexer);
  }

  public final void intakeNode() {
    new ArmToPosition(m_arm, ArmSubsystem.armPositions.INTAKE);

    new RunIntake(m_intake);
    new RunIndexerShooter(m_indexer);
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
    m_driverController.x().onTrue(new InstantCommand(() -> m_robotDrive.zeroHaw()));
    //m_manipController.button(5).whileTrue(new manipIntake(m_arm, m_intake, m_feeder, m_indexer));
    //m_manipController.button(6).whileTrue(new stowArm(m_arm));

    //m_manipController.button(4).onTrue(new ArmToPosition(m_arm, ArmSubsystem.armPositions.INTAKE));
    //m_manipController.button(2).onTrue(new ArmToPosition(m_arm, ArmSubsystem.armPositions.AMP));
    //m_manipController.button(1).onTrue(new ArmToPosition(m_arm, ArmSubsystem.armPositions.STOWED));
    //m_manipController.button(3).onTrue(new ArmToPosition(m_arm, ArmSubsystem.armPositions.SOURCE));
    //Intake Note
    m_driverController.button(6).whileTrue(new ParallelCommandGroup(new ArmToPosition(m_arm, armPositions.INTAKE), new intakeFromFloor(m_intake, m_feeder, m_indexer))).onFalse(new ArmToPosition(m_arm, armPositions.INTAKE));
    //Stow arm
    m_driverController.button(5).onTrue(new ArmToPosition(m_arm, armPositions.INTAKE));

    m_driverController.button(2).whileTrue(new findColor(m_indexer));

    Trigger readyToShoot = 
      new Trigger(
      () -> m_shooter.atGoal() && m_arm.atPosition());

    m_manipController
        .button(2)
        .and(readyToShoot)
        .whileTrue(
            Commands.startEnd(
                () -> {
                  m_manipController.getHID().setRumble(RumbleType.kLeftRumble, 1.0);
                },
                () -> {
                  m_manipController.getHID().setRumble(RumbleType.kBothRumble, 0.0);
                }));
    
                
    m_manipController
        .button(3)
        .and(readyToShoot)
        .whileTrue(
            Commands.startEnd(
                () -> {
                  m_manipController.getHID().setRumble(RumbleType.kLeftRumble, 1.0);
                },
                () -> {
                  m_manipController.getHID().setRumble(RumbleType.kBothRumble, 0.0);
                }));

    //Intake from source
    m_manipController.button(1).whileTrue(new ParallelCommandGroup(new ArmToPosition(m_arm, armPositions.SOURCE), new RunIndexerAmp(m_indexer), new RunShooterReverse(m_shooter)));
    //Stow arm
    m_manipController.button(5).onTrue(new ArmToPosition(m_arm, armPositions.INTAKE));
    //Shoot into speaker from sub
    m_manipController.button(2).whileTrue(new ParallelCommandGroup(new ArmToPosition(m_arm, armPositions.SUBSHOT), 
    m_shooter.shootCommand())).onFalse(new ArmToPosition(m_arm, armPositions.SUBSHOT));//.and(m_manipController.button(6)).whileTrue(new SequentialCommandGroup
    //(new ReverseFeeder(m_feeder), new ParallelCommandGroup(new RunFeeder(m_feeder), new RunIndexerShooter(m_indexer))));

    m_manipController.button(3).whileTrue(new ParallelCommandGroup(new ArmToPosition(m_arm, armPositions.PODSHOT), m_shooter.shootCommand())).onFalse(new ArmToPosition(m_arm, armPositions.PODSHOT));
    //Shoot into speaker from pod
    //m_manipController.button(3).whileTrue(new ParallelCommandGroup(new ArmToPosition(m_arm, armPositions.PODSHOT), 
    //new RunShooter(m_shooter))).and(m_manipController.button(6).whileTrue(new SequentialCommandGroup
    //(new ReverseFeeder(m_feeder), new ParallelCommandGroup(new RunFeeder(m_feeder), new RunIndexerShooter(m_indexer)))));

    m_manipController.button(6).whileTrue(new SequentialCommandGroup(new feedToIndexer(m_feeder), (new ParallelCommandGroup(new RunFeeder(m_feeder), new RunIndexerShooter(m_indexer)))));

    //Unjam deadzone
    m_manipController.pov(0).whileTrue(new ParallelCommandGroup(new ReverseFeeder(m_feeder), new ArmToPosition(m_arm, armPositions.AMP))).onFalse(new ArmToPosition(m_arm, armPositions.AMP));
    //Unjam intake
    m_manipController.pov(90).whileTrue(new ParallelCommandGroup(new RunIntakeUnjam(m_intake), new ArmToPosition(m_arm, armPositions.INTAKE), new ReverseFeeder(m_feeder))).onFalse(new ArmToPosition(m_arm, armPositions.INTAKE));
    //Unjam  shooter
    m_manipController.pov(180).whileTrue(new ParallelCommandGroup(new RunShooterReverse(m_shooter), new RunIndexerAmp(m_indexer)));

    //m_manipController.button(5).whileTrue(new InstantCommand( )
    //m_manipController.button(5).whileTrue(new RunIndexer(m_indexer));
    //m_manipController.button(2).whileTrue(new RunIntake(m_intake));
  }
}
