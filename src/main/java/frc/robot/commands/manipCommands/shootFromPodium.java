package frc.robot.commands.manipCommands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.arm.ArmHoldPosition;
import frc.robot.commands.arm.ArmToPosition;
import frc.robot.commands.feeder.RunFeeder;
import frc.robot.commands.indexer.RunIndexerShooter;
import frc.robot.commands.shooter.RunShooter;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.ArmSubsystem.armPositions;

public class shootFromPodium {
    public shootFromPodium(ArmSubsystem arm, FeederSubsystem feeder, IndexerSubsystem indexer, ShooterSubsystem shooter)
    {
        new SequentialCommandGroup(
            new ArmToPosition(arm, armPositions.PODSHOT).alongWith(new RunShooter(shooter)),
                new ParallelCommandGroup(
                    new ArmHoldPosition(arm, armPositions.PODSHOT),
                    new RunShooter(shooter),
                    new RunFeeder(feeder),
                    new RunIndexerShooter(indexer)
                )
        );
    }
}
