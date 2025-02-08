package frc.robot.commands.Coral;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Coral;

public class OuttakeCommand extends Command { 

    private final Coral coralSubsystem;
    //private static DigitalInput linebreak;
    

    //constructor for OuttakeCommand
    public OuttakeCommand(Coral coralSubsystem) {
        this.coralSubsystem = coralSubsystem; 

        addRequirements(coralSubsystem);
    }
    
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        if(coralSubsystem.isLineBroken()) {
            coralSubsystem.set(Constants.coralConstants.outtakeSpeed);
        }else {
            cancel();
        }
    }

    //execute, startOuttake() once button is pressed
    @Override
    public void execute() {

    } 
        
    //end, when command ends, set activeOuttake to false and set state to IDLE
    @Override
    public void end(boolean interrupted) {
        coralSubsystem.setIdle();
    }

     //if line is not broken, return true, else return false
    @Override
    public boolean isFinished() {
        if(!coralSubsystem.isLineBroken()){
            return true;
        }
        return false;
    }
}