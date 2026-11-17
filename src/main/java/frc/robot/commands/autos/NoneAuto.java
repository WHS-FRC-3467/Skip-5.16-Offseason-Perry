package frc.robot.commands.autos;

import frc.robot.commands.autos.utils.AutoOption;
import frc.robot.commands.autos.utils.AutoUtil;

import lombok.AccessLevel;
import lombok.NoArgsConstructor;

import org.wpilib.command2.Commands;

@NoArgsConstructor(access = AccessLevel.PRIVATE)
public final class NoneAuto {
    public static AutoOption create() {
        return AutoUtil.commandOption(Commands::none);
    }
}
