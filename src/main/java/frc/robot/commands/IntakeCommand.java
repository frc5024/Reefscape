
package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Coral;
import frc.robot.subsystems.Coral.coralState;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants;


public class IntakeCommand extends Command { 

    private final Coral coralSubsystem;
    private static DigitalInput linebreak;
    //private static DigitalInput linebreakBottom;
    
    //constructor for IntakeCommand
    public IntakeCommand(Coral coralSubsystem) {
        this.coralSubsystem = coralSubsystem;
        
        linebreak = new DigitalInput(Constants.coralConstants.linebreakChannel);
        //linebreakBottom = new DigitalInput(Constants.coralConstants.linebreakBottomChannel);

        addRequirements(coralSubsystem);
    }
    
    //initizlize, when command starts, set activeIntake to true, start timer, but state should be IDLE
    @Override
    public void initialize() {
        activeIntake(true);
        coralSubsystem.state = coralState.IDLE;

    }
    //execute, if line is broken, and timer is greater than 0.05, set activeIntake to false, and state to HOLDING (you want motors to stop)
    @Override
    public void execute() {
        if(isLineBroken()){
            
                activeIntake(false);
                coralSubsystem.state = coralState.HOLDING;
            
        }
    }
    //end, when command ends, set activeIntake to false
    @Override
    public void end(boolean interrupted) {
        activeIntake(false);
    }

    public static boolean isLineBroken() {
        return linebreak.get();
    }

    //always return false for isFinished()
    @Override
    public boolean isFinished() {
        return false;
    }
    //method for activeIntake, if line is not broken and intaking is true, start intake, else, start intake to false
    public void activeIntake(boolean intaking) {
        if(!isLineBroken() && intaking) {
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