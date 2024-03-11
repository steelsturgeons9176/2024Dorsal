package frc.robot.commands.manipCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.arm.ArmToPosition;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ArmSubsystem.armPositions;

public class stowArm extends Command {
    public stowArm(ArmSubsystem arm)
    {
        addRequirements(arm);
        new ArmToPosition(arm, armPositions.STOWED);
    }

}
