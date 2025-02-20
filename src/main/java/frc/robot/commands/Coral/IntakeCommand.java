
package frc.robot.commands.Coral;

import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.leds.LEDPreset;
import frc.robot.Constants;
import frc.robot.subsystems.Coral;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.Rumble;

public class IntakeCommand extends Command {

    private final Coral coralSubsystem;

    private final LEDs s_LEDs = LEDs.getInstance();
    Rumble rumble = new Rumble();

    // constructor for IntakeCommand
    public IntakeCommand(Coral coralSubsystem) {
        this.coralSubsystem = coralSubsystem;

        addRequirements(coralSubsystem);
    }

    // initialize, when command starts, if line is not broken, set state to IDLE
    @Override
    public void initialize() {
        if (!coralSubsystem.isLineBroken()) {
            coralSubsystem.set(Constants.coralConstants.intakeSpeed);
            s_LEDs.setCommand(LEDPreset.Strobe.kGold).schedule();
        }
    }

    // execute, if button is pressed, startIntake()
    @Override
    public void execute() {

    }

    // end, when command ends, set Idle
    @Override
    public void end(boolean interrupted) {
        coralSubsystem.setIdle();
        s_LEDs.setCommand(LEDPreset.Solid.kRed).schedule();
        if (coralSubsystem.isLineBroken()) {
            rumble.staticRumble(true);
        }
    }

    // if line is broken, return true, else return false
    @Override
    public boolean isFinished() {
        return coralSubsystem.isLineBroken();
    }

}