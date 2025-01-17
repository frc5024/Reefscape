
package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Coral;
import frc.robot.subsystems.Coral.coralState;
import edu.wpi.first.wpilibj.Timer; 
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants;


public class IntakeCommand extends Command { 

    private final Coral coralSubsystem;
    private final Timer timer;
    private static DigitalInput linebreak;
    

    public IntakeCommand(Coral coralSubsystem) {
        this.coralSubsystem = coralSubsystem;
        
        linebreak = new DigitalInput(Constants.coralConstants.linebreakChannel);
        timer = new Timer();


        addRequirements(coralSubsystem);
    }
    

    @Override
    public void initialize() {
        activeIntake(true);
        timer.reset();
        timer.start();
        coralSubsystem.state = coralState.IDLE;

    }

    @Override
    public void execute() {
        if(isLineBroken()){
            if(timer.get() >= 0.05) {
                activeIntake(false);
                coralSubsystem.state = coralState.HOLDING;
            }
        }
    }
    
    @Override
    public void end(boolean interrupted) {
        activeIntake(false);
    }

    public static boolean isLineBroken() {
        return linebreak.get();
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    public void activeIntake(boolean intaking) {
        if(!isLineBroken() && intaking) {
            coralSubsystem.startIntake(true);
            coralSubsystem.state = coralState.IDLE;
        }
        else {
            coralSubsystem.startIntake(false);
            coralSubsystem.state = coralState.HOLDING;
        }
    }
}