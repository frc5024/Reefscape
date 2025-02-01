package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Coral;
import frc.robot.subsystems.Coral.coralState;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;


public class OuttakeCommand extends Command { 

    private final Coral coralSubsystem;
    private final Timer outtakeTimer;
    private static DigitalInput linebreak;
    
    //constants for outtakeTime
    double outtakeTime = Constants.coralConstants.outtakeTime;

    //constructor for OuttakeCommand
    public OuttakeCommand(Coral coralSubsystem) {
        this.coralSubsystem = coralSubsystem; 

        linebreak = new DigitalInput(Constants.coralConstants.linebreakChannel);
        outtakeTimer = new Timer();

        addRequirements(coralSubsystem);
    }
    
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        if(IntakeCommand.isLineBroken()) {
            coralSubsystem.state = coralState.HOLDING;
            outtakeTimer.reset();
            outtakeTimer.start();
        }

        activeOuttake(false);

    }
    //execute, if line is not broken, and t2nd linebreak not broken, set activeOuttake to false and state to IDLE
    @Override
    public void execute() {
        if(!isLineBroken()) {
            if(outtakeTimer.get() >= outtakeTime) {
                activeOuttake(true);
                coralSubsystem.state = coralState.IDLE;
                coralSubsystem.setIdle();
                cancel();
            }
        }
    }
        
    //end, when command ends, set activeOuttake to false and set state to IDLE
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

    //method for activeOuttake, 
    public void activeOuttake(boolean outtaking) {
        //if line is broken and outtaking is true, start outtake and set state to DROPPING
        if(isLineBroken() && outtaking) {
            coralSubsystem.startOuttake(true);
            coralSubsystem.state = coralState.DROPPING;
        }
        //else, stop outtake and set state to IDLE
        else {
            coralSubsystem.startOuttake(false);
            coralSubsystem.state = coralState.IDLE;
        }
    }
}