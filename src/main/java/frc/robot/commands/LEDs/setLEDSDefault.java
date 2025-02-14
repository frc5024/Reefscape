package frc.robot.commands.LEDs;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LEDs;

public class SetLEDSDefault extends Command {
    private LEDs leds;

    // Constructor, intakes LED Subsystem
    public SetLEDSDefault(LEDs leds) {
        this.leds = leds;
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        // Sets LEDs to default colour
        leds.setDefault();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return true;
    }
}
