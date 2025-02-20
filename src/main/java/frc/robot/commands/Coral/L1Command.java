package frc.robot.commands.Coral;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Coral;
import frc.robot.Constants;


public class L1Command extends Command { 

    private final Coral coralSubsystem;

    //constructor for plopCommand
    public L1Command(Coral coralSubsystem) {
        this.coralSubsystem = coralSubsystem; 

        addRequirements(coralSubsystem);
    }
    
    // Called when the command is initially scheduled, if line is broken, set state to HOLDING
    @Override
    public void initialize() {
        coralSubsystem.set(Constants.coralConstants.L1Speed);
        
    }
    //execute, once button is pressed, startPlop()
    @Override
    public void execute() {
       
    } 
        
    //end, when command ends, set IDLE
    @Override
    public void end(boolean interrupted) {
        coralSubsystem.setIdle();
    }
    //if line is not broken, return true, else return false
    @Override
    public boolean isFinished() {
        return !coralSubsystem.isLineBroken();
    }
}