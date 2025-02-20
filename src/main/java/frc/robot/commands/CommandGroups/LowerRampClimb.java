package frc.robot.commands.CommandGroups;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Coral;
import frc.robot.subsystems.Climb;
import frc.robot.commands.Climb.ClimbExtendoCommand;

public class LowerRampClimb extends SequentialCommandGroup {
    
    public LowerRampClimb(Coral coralSubsystem, Climb climbSubsystem) {
        addCommands(coralSubsystem.lowerRampCommand(),
                new ClimbExtendoCommand(climbSubsystem, true));
        
    }
    
}
