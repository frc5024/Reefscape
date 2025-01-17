package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Coral;
import frc.robot.subsystems.Coral.coralState;
import edu.wpi.first.wpilibj.Timer; 
import frc.robot.Constants;
import edu.wpi.first.wpilibj.DigitalInput;


public class OuttakeCommand extends Command { 

    private final Coral coralSubsystem;
    private final Timer timer;
    private static DigitalInput linebreak;
    
    int outtakeTime = Constants.outtakeConstants.outtakeTime;

    public OuttakeCommand(Coral coralSubsystem) {
        this.coralSubsystem = coralSubsystem;
        timer = new Timer();
        linebreak = new DigitalInput(Constants.coralConstants.linebreakChannel);

        addRequirements(coralSubsystem);
    }
    
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        if(IntakeCommand.isLineBroken()) {
            coralSubsystem.state = coralState.HOLDING;
        }

        activeOuttake(false);
        timer.reset();
        timer.start();

    }

    @Override
    public void execute() {
        if(!isLineBroken()) {
            if(timer.get() >= 0.5) {
                activeOuttake(false);
                coralSubsystem.state = coralState.IDLE;
                coralSubsystem.setIdle();
            }
        }
    }
        
          
    @Override
    public void end(boolean interrupted) {
        activeOuttake(false);
        coralSubsystem.state = coralState.IDLE;
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    public static boolean isLineBroken() {
        return linebreak.get();
    }

    public void activeOuttake(boolean outtaking) {
        if(isLineBroken() && outtaking) {
            coralSubsystem.startOuttake(true);
            coralSubsystem.state = coralState.DROPPING;
        }
        else {
            coralSubsystem.startOuttake(false);
            coralSubsystem.state = coralState.IDLE;
        }
    }
}