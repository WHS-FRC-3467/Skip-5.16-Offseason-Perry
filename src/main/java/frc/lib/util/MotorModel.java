/*
 * Copyright (C) 2026 Windham Windup
 *
 * This program is free software: you can redistribute it and/or modify it under the terms of the
 * GNU General Public License as published by the Free Software Foundation, either version 3 of the
 * License, or any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without
 * even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with this program. If
 * not, see <https://www.gnu.org/licenses/>.
 */
package frc.lib.util;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.NewtonMeters;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Watts;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Power;
import edu.wpi.first.units.measure.Torque;

/**
 * Describes the performance attributes of various motors for use in diagnostics, sim, and controls
 * limits.
 */
public enum MotorModel {
    KRAKEN_X60_FOC(NewtonMeters.of(9.37), Amps.of(483.0), RotationsPerSecond.of(96.6), Amps.of(2.0), Watts.of(1405.0), 85.4),
    KRAKEN_X44_FOC(NewtonMeters.of(5.01), Amps.of(329.0), RotationsPerSecond.of(122.8), Amps.of(3.0), Watts.of(966.0), 81.0);

    private final Torque STALL_TORQUE;
    private final Current STALL_CURRENT;
    private final AngularVelocity FREE_SPEED;
    private final Current FREE_CURRENT; 
    private final Power PEAK_POWER;
    private final double MAX_EFFICIENCY;

    private MotorModel(Torque STALL_TORQUE, Current STALL_CURRENT, AngularVelocity FREE_SPEED, Current FREE_CURRENT, Power PEAK_POWER, double MAX_EFFICIENCY) {
        this.STALL_TORQUE = STALL_TORQUE;
        this.STALL_CURRENT = STALL_CURRENT;
        this.FREE_SPEED = FREE_SPEED;
        this.FREE_CURRENT = FREE_CURRENT; 
        this.PEAK_POWER = PEAK_POWER;
        this.MAX_EFFICIENCY = MAX_EFFICIENCY;
    }

    /**
     * Maximum rotational force the motor can exert when the shaft is completely locked (i.e. 0
     * RPM).
     */
    public Torque getStallTorque() {
        return STALL_TORQUE;
    }

    /**
     * The amount of electrical current the motor draws from the battery when it is powered but
     * completely locked (i.e. 0 RPM).
     */
    public Current getStallCurrent() {
        return STALL_CURRENT;
    }

    /**
     * Maximum rotational speed the motor reaches when running with absolutely no load attached --
     * back EMF = forward voltage.
     */
    public AngularVelocity getFreeSpeed() {
        return FREE_SPEED;
    }

    /** The current the motor draws when spinning freely at maximum speed, consumed in its entirety to overcome internal losses. */
    public Current getFreeCurrent() {
        return FREE_CURRENT;
    }

    /** The maximum mechanical work the motor can perform per second = max(T * omega) */
    public Power getPeakPower() {
        return PEAK_POWER;
    }

    /** Peak ratio of mechanical output power to electrical input power. */
    public double getMaxEfficiency() {
        return MAX_EFFICIENCY;
    }

}
