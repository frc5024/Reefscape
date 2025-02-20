package frc.robot.commands.CommandGroups;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Coral;
import frc.robot.subsystems.Elevator;
import frc.robot.Constants;
import frc.robot.commands.SetElevatorSetpointCmd;

public class CoralScored extends SequentialCommandGroup {
        
    public CoralScored(Coral coralSubsystem, Elevator elevatorSubsystem) {

        addCommands(coralSubsystem.outtakeCommand(),
             new SetElevatorSetpointCmd(elevatorSubsystem, Constants.elevatorConstants.rootPosition));
    }
}
    
    

