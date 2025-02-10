
package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Coral;


public class testCommand extends Command { 

    private final Coral coralSubsystem;
    //private static DigitalInput linebreakBottom;
    
    //constructor for IntakeCommand
    public testCommand(Coral coralSubsystem) {
        this.coralSubsystem = coralSubsystem;

        addRequirements(coralSubsystem);
    }
    
    //initialize, when command starts, if line is not broken, set state to IDLE
    @Override
    public void initialize() {
        coralSubsystem.set(0);

    }
    //execute, if button is pressed, startIntake()
    @Override
    public void execute() {
        coralSubsystem.set(Constants.coralConstants.intakeSpeed);
        
    } 
    
    //end, when command ends, set Idle
    @Override
    public void end(boolean interrupted) {
        
    }


    //if line is broken, return true, else return false
    @Override
    public boolean isFinished() {
        return false;
    }
    
}