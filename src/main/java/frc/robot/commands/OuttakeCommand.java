package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Coral;
import frc.robot.subsystems.Coral.coralState;
import frc.robot.Constants;


public class OuttakeCommand extends Command { 

    private final Coral coralSubsystem;
    //private static DigitalInput linebreak;
    
    //constants for outtakeTime
    double outtakeTime = Constants.coralConstants.outtakeTime;

    //constructor for OuttakeCommand
    public OuttakeCommand(Coral coralSubsystem) {
        this.coralSubsystem = coralSubsystem; 

        addRequirements(coralSubsystem);
    }
    
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        if(coralSubsystem.isLineBroken()) {
            coralSubsystem.state = coralState.HOLDING;
        }
    }
    //execute, if line is not broken, and t2nd linebreak not broken, set activeOuttake to false and state to IDLE
    @Override
    public void execute() {
       coralSubsystem.startOuttake();
    } 
        
    //end, when command ends, set activeOuttake to false and set state to IDLE
    @Override
    public void end(boolean interrupted) {
        coralSubsystem.setIdle();
    }

    @Override
    public boolean isFinished() {
        if(!coralSubsystem.isLineBroken()){
            return true;
        }
        return false;
    }
}