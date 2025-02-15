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
        System.out.println(startingValue);
        if(!coralSubsystem.isLineBroken()) {
            coralSubsystem.setBottom(-Constants.coralConstants.intakeSpeed);
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
        return false;
        // if(coralSubsystem.getEncoder() > startingValue + 1792){ //1 rotation = 7168 ticks
        //     return true;
        // } else {
        //     return false;
        // }
    }
}
