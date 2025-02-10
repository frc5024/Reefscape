
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
        
        // if(!coralSubsystem.isLineBroken()) {
        //     System.out.println("THELINE IS BROKEN THE LINE IS BROKENNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNN");
        //     coralSubsystem.set(Constants.coralConstants.intakeSpeed);
        // }else{
        //     cancel();
        // }

    }
    //execute, if button is pressed, startIntake()
    @Override
    public void execute() {
        
        if(coralSubsystem.isLineBroken()) {
            coralSubsystem.setIdle();
            cancel();
        } else {
            System.out.println("HHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHH");
            coralSubsystem.set(Constants.coralConstants.intakeSpeed);
        }
        //System.out.println("hhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhh");
    } 
    
    //end, when command ends, set Idle
    @Override
    public void end(boolean interrupted) {
        coralSubsystem.setIdle();
    }


    //if line is broken, return true, else return false
    @Override
    public boolean isFinished() {
        if(coralSubsystem.isLineBroken()){
            System.out.println("THELINE IS BROKEN THE LINE IS BROKENNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNN");
            return true;
        }
        return false;
    }
    
}