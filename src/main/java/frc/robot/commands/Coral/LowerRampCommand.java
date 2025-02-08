package frc.robot.commands.Coral;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Coral;

public class LowerRampCommand extends Command {

    private final Coral coralSubsystem;
    private double startingValue;

    public LowerRampCommand(Coral coralSubsystem) {
        
        this.coralSubsystem = coralSubsystem; 

        addRequirements(coralSubsystem);
    }

    //when start, rotate -plopspeed, lowers the ramp
    public void initialize() {
        startingValue = coralSubsystem.getEncoder();
        if(!coralSubsystem.isLineBroken()) {
            coralSubsystem.set(-Constants.coralConstants.rampSpeed);
        }
    }

    //when button pressed, rotate servo 90 degrees (lowers the ramp)
    public void execute() {
    }
    
    //when command ends, set idle
    @Override
    public void end(boolean interrupted) {
       coralSubsystem.setIdle();
    }

    // Returns true when the command should end. 
    @Override
    public boolean isFinished() {
        if(coralSubsystem.getEncoder() > startingValue + 90){
            return true;
        } else {
            return false;
        }
    }
}
