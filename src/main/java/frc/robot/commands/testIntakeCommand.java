package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Coral;
import frc.robot.subsystems.Coral.coralState;


public class testIntakeCommand extends Command { 

    private final Coral coralSubsystem;
    
    //constructor for IntakeCommand
    public testIntakeCommand(Coral coralSubsystem) {
        this.coralSubsystem = coralSubsystem;
        

        addRequirements(coralSubsystem);
    }
    
    //initialize, when command starts, set activeIntake to true, start timer, but state should be IDLE
    @Override
    public void initialize() {
        activeIntake(true);
        coralSubsystem.state = coralState.IDLE;

    }
    //execute, if line is broken, and timer is greater than 0.05, set activeIntake to false, and state to HOLDING (you want motors to stop)
    @Override
    public void execute() {
        coralSubsystem.state = coralState.HOLDING;
        
}
    
    //end, when command ends, set activeIntake to false
    @Override
    public void end(boolean interrupted) {
        activeIntake(false);
    }

    //always return false for isFinished()
    @Override
    public boolean isFinished() {
        return false;
    }
    //method for activeIntake, if line is not broken and intaking is true, start intake, else, start intake to false
    public void activeIntake(boolean intaking) {
        if(intaking) {
            coralSubsystem.startIntake(true);
            coralSubsystem.state = coralState.IDLE;
        }
        //if line is broken, stop intake
        else {
            coralSubsystem.startIntake(false);
            coralSubsystem.state = coralState.HOLDING;
        }
    }
}
