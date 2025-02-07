
package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Coral;
import frc.robot.subsystems.Coral.coralState;


public class IntakeCommand extends Command { 

    private final Coral coralSubsystem;
    //private static DigitalInput linebreakBottom;
    
    //constructor for IntakeCommand
    public IntakeCommand(Coral coralSubsystem) {
        this.coralSubsystem = coralSubsystem;

        addRequirements(coralSubsystem);
    }
    
    //initizlize, when command starts, set activeIntake to true, start timer, but state should be IDLE
    @Override
    public void initialize() {
        if(!coralSubsystem.isLineBroken()) {
            coralSubsystem.state = coralState.IDLE;
        }

    }
    //execute, if line is broken, and timer is greater than 0.05, set activeIntake to false, and state to HOLDING (you want motors to stop)
    @Override
    public void execute() {
        coralSubsystem.startIntake();
    } 
    
    //end, when command ends, set activeIntake to false
    @Override
    public void end(boolean interrupted) {
        coralSubsystem.setIdle();
    }


    //always return false for isFinished()
    @Override
    public boolean isFinished() {
        if(coralSubsystem.isLineBroken()){
            return true;
        }
        return false;
    
    }
    
}