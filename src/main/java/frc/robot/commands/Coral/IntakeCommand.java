
package frc.robot.commands.Coral;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Coral;


public class IntakeCommand extends Command { 

    private final Coral coralSubsystem;
    //private static DigitalInput linebreakBottom;
    
    //constructor for IntakeCommand
    public IntakeCommand(Coral coralSubsystem) {
        this.coralSubsystem = coralSubsystem;

        addRequirements(coralSubsystem);
    }
    
    //initialize, when command starts, if line is not broken, set state to IDLE
    @Override
    public void initialize() {
            coralSubsystem.set(Constants.coralConstants.intakeSpeed);

    }
    //execute, if button is pressed, startIntake()
    @Override
    public void execute() {
        
    } 
    
    //end, when command ends, set Idle
    @Override
    public void end(boolean interrupted) {
        coralSubsystem.setIdle();
    }


    //if line is broken, return true, else return false
    @Override
    public boolean isFinished() {
       
        return coralSubsystem.isLineBroken();
    }
    
}