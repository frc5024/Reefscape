
package frc.robot.commands.Coral;

import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.leds.LEDPreset;
import frc.robot.Constants;
import frc.robot.subsystems.Coral;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.Rumble;

public class IntakeCommand extends Command {

    private final Coral coralSubsystem;
    private final Command ledCmd;

    private final LEDs s_LEDs = LEDs.getInstance();
    Rumble rumble = Rumble.getInstance();

    // constructor for IntakeCommand
    public IntakeCommand(Coral coralSubsystem) {
        this.coralSubsystem = coralSubsystem;

        ledCmd = s_LEDs.persistCommand(LEDPreset.Strobe.kRed);

        addRequirements(coralSubsystem);
    }

    // initialize, when command starts, if line is not broken, set state to IDLE
    @Override
    public void initialize() {
        if (!coralSubsystem.isLineBroken()) {
            coralSubsystem.set(Constants.coralConstants.intakeSpeed);
        }

        ledCmd.schedule();
    }

    // execute, if button is pressed, startIntake()
    @Override
    public void execute() {
    }

    // end, when command ends, set Idle
    @Override
    public void end(boolean interrupted) {
        coralSubsystem.setIdle();

        if (!interrupted)
            rumble.staticRumble(true);

        ledCmd.cancel();
    }

    // if line is broken, return true, else return false
    @Override
    public boolean isFinished() {
        return coralSubsystem.isLineBroken();
    }

}