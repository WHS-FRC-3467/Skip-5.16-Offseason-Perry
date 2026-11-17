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

package frc.lib.io.servo;

import static org.wpilib.units.Units.Degrees;

import frc.lib.util.Device;

import org.wpilib.hardware.discrete.PWM;
import org.wpilib.units.measure.Angle;

/**
 * ServoIOPWM drives a hobby servo with Pulse Width Modulation signal control (PWM) and implements
 * the ServoIO interface. WPILib 2027 removed the Servo class, so this maps positions onto raw PWM
 * pulse widths using the same 0.6ms-2.4ms range the 2026 Servo class used. It provides methods to
 * run the servo to a specified position.
 */
public class ServoIOPWM implements ServoIO {

    // Pulse range used by the 2026 WPILib Servo class (Hitec HS-322HD defaults)
    private static final int MIN_PULSE_US = 600;
    private static final int MAX_PULSE_US = 2400;

    private final PWM servo;

    private double minAngle;
    private double maxAngle;

    public ServoIOPWM(Device.PWM id, Angle minAngle, Angle maxAngle) {
        servo = new PWM(id.id());
        servo.setOutputPeriod(20);
        this.minAngle = minAngle.in(Degrees);
        this.maxAngle = maxAngle.in(Degrees);
    }

    /**
     * Set the servo position.
     *
     * @param value Position from 0.0 to 1.0, corresponding to the range of full left to full right.
     */
    public void setScaledPosition(double value) {
        setPulseFraction(value);
    }

    /**
     * Set the servo angle.
     *
     * <p>Servo angles that are out of the supported range of the servo simply "saturate" in that
     * direction In other words, if the servo has a range of (X degrees to Y degrees) than angles of
     * less than X result in an angle of X being set and angles of more than Y degrees result in an
     * angle of Y being set.
     *
     * @param degrees The angle in degrees to set the servo.
     */
    public void setAngle(double degrees) {
        var servoAngleRange = maxAngle - minAngle;
        setPulseFraction((Math.clamp(degrees, minAngle, maxAngle) - minAngle) / servoAngleRange);
    }

    private void setPulseFraction(double value) {
        double clamped = Math.clamp(value, 0.0, 1.0);
        servo.setPulseTimeMicroseconds(
                (int) Math.round(MIN_PULSE_US + clamped * (MAX_PULSE_US - MIN_PULSE_US)));
    }

    /**
     * Set the servo angle.
     *
     * <p>Servo angles that are out of the supported range of the servo simply "saturate" in that
     * direction In other words, if the servo has a range of (X degrees to Y degrees) than angles of
     * less than X result in an angle of X being set and angles of more than Y degrees result in an
     * angle of Y being set.
     *
     * @param angle The Angle set the servo.
     */
    @Override
    public void setAngle(Angle angle) {
        setAngle(angle.in(Degrees));
    }

    /** Disables the PWM output until told to run to a position again. */
    @Override
    public void stop() {
        servo.setDisabled();
    }
}
