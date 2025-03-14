// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.LEDs;

import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.leds.LEDPreset;
import frc.robot.subsystems.Coral;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.Limelight;

public class LEDDefaultCmd extends Command {

    private final LEDs s_LED;
    Coral coralSubsystem = Coral.getInstance();
    Limelight s_Limelight = Limelight.getInstance();

    public LEDDefaultCmd(LEDs s_LED) {
        this.s_LED = s_LED;

        addRequirements(s_LED);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        if (coralSubsystem.isLineBroken()) {
            s_LED.set(LEDPreset.Solid.kGreen);
        } else {
            s_LED.set(LEDPreset.Solid.kRed);
        }
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
