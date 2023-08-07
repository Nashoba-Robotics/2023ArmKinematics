package frc.robot.commands.autoscore;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.GrabberSubsystem;

public class AutoHighConeCommand extends SequentialCommandGroup{
    public AutoHighConeCommand(){
        addCommands(
            new InstantCommand(()->{
                GrabberSubsystem.getInstance().zeroWrist();
            }),
            new HighSetCommand().withTimeout(1.05),
            // new WaitCommand(5),
            new ScoreCommand().withTimeout(2 ),
            new InstantCommand(() -> GrabberSubsystem.getInstance().setOrientSpeed(0))
        );
    }
}
