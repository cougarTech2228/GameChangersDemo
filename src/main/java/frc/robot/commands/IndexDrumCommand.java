package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.StorageSubsystem;
import frc.robot.Constants;
import frc.robot.OI;
import frc.robot.Toolkit.CT_DigitalInput;
import frc.robot.subsystems.ShooterSubsystem;

/**
 * IndexCommand
 * 
 * Indexes the drum TODO
 */
public class IndexDrumCommand extends SequentialCommandGroup {
    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })

    public IndexDrumCommand(StorageSubsystem storageSubsystem, CT_DigitalInput input, boolean ignoreCell) {
        
        addCommands(
            new InstantCommand(() -> {
                System.out.println("Index drum because cell input interrupted");

                if(storageSubsystem.getCellInput().getStatus() && !ignoreCell) {
                    System.out.println("Acquiring ball");
                    storageSubsystem.getBallArray().acquire();
                }
                storageSubsystem.getBallArray().rotate();

                storageSubsystem.startDrumMotor();
            }),
            new WaitCommand(0.2),
            new InstantCommand(() -> input.setInterruptLatched(true))
        );
    }
}
