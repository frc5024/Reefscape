// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.LEDs;

import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.leds.ILEDPreset;
import frc.robot.subsystems.LEDs;

public class persistLEDSCmd extends Command {
    private LEDs leds;
    private ILEDPreset colour;

    // Constructor, intakes LED subsystem and LED Preset colour
    public persistLEDSCmd(LEDs leds, ILEDPreset colour) {
        this.leds = leds;
        this.colour = colour;
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        leds.set(colour); // sets to colour based on constructor intake
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
