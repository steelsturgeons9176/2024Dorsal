package frc.robot.commands.manipCommands;

import frc.robot.Constants.ArmConstants;
import frc.robot.commands.arm.ArmHoldPosition;
import frc.robot.commands.arm.ArmToPosition;
import frc.robot.commands.intake.RunIntake;
import frc.robot.commands.feeder.RunFeeder;
import frc.robot.commands.indexer.RunIndexerShooter;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ArmSubsystem.armPositions;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.arm.ArmToPosition;
public class manipIntake extends Command {

    public manipIntake(ArmSubsystem arm, IntakeSubsystem intake, FeederSubsystem feeder, IndexerSubsystem indexer){

        addRequirements(arm);
        addRequirements(intake);
        addRequirements(feeder);
        addRequirements(indexer);


        new ParallelCommandGroup(
            new ArmToPosition(arm, armPositions.INTAKE).andThen(new ArmHoldPosition(arm, armPositions.INTAKE)), 
            new RunIntake(intake),
            new RunFeeder(feeder),
            new RunIndexerShooter(indexer)
        );
    }




}