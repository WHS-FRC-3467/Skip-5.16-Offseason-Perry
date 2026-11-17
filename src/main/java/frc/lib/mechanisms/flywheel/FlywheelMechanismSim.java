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

package frc.lib.mechanisms.flywheel;

import static org.wpilib.units.Units.KilogramSquareMeters;
import static org.wpilib.units.Units.Radians;
import static org.wpilib.units.Units.RadiansPerSecond;
import static org.wpilib.units.Units.RadiansPerSecondPerSecond;
import static org.wpilib.units.Units.Seconds;
import static org.wpilib.units.Units.Volts;

import frc.lib.io.motor.MotorIOSim;

import org.wpilib.math.system.DCMotor;
import org.wpilib.math.system.Models;
import org.wpilib.simulation.BatterySim;
import org.wpilib.simulation.FlywheelSim;
import org.wpilib.simulation.RoboRioSim;
import org.wpilib.system.RobotController;
import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.AngularVelocity;
import org.wpilib.units.measure.MomentOfInertia;
import org.wpilib.units.measure.Time;
import org.wpilib.util.Color;

/**
 * A simulated implementation of the FlywheelMechanism abstract class that uses FlywheelSim to
 * simulate the behavior of a flywheel mechanism.
 */
public class FlywheelMechanismSim extends FlywheelMechanism<MotorIOSim> {
    private final FlywheelSim sim;
    private final FlywheelVisualizer visualizer;
    private final AngularVelocity tolerance;

    private Time lastTime = RobotController.getMeasureTime();

    public FlywheelMechanismSim(
            String name,
            MotorIOSim io,
            DCMotor motor,
            MomentOfInertia momentOfInertia,
            AngularVelocity tolerance) {
        super(name, io);

        if (momentOfInertia.isEquivalent(KilogramSquareMeters.zero()))
            throw new IllegalArgumentException("momentOfInertia must be greater than zero!");

        this.tolerance = tolerance;
        sim =
                new FlywheelSim(
                        Models.flywheelFromPhysicalConstants(
                                motor,
                                momentOfInertia.in(KilogramSquareMeters),
                                io.getRotorToSensorRatio() * io.getSensorToMechanismRatio()),
                        motor);

        visualizer = new FlywheelVisualizer(name);
    }

    @Override
    public void periodic() {
        Time currentTime = RobotController.getMeasureTime();
        double deltaTime = currentTime.minus(lastTime).in(Seconds);

        sim.setInputVoltage(inputs.appliedVoltage.in(Volts));
        sim.update(deltaTime);
        RoboRioSim.setVInVoltage(
                BatterySim.calculateDefaultBatteryLoadedVoltage(sim.getCurrentDraw()));

        lastTime = currentTime;

        io.setRotorVelocity(RadiansPerSecond.of(sim.getAngularVelocity()));
        io.setRotorAcceleration(RadiansPerSecondPerSecond.of(sim.getAngularAcceleration()));

        // Angular displacement kinematic equation (θ = ω₀t + (1/2)αt²)'
        Angle positionChange =
                Radians.of(
                        sim.getAngularVelocity() * deltaTime
                                + 0.5 * sim.getAngularAcceleration() * Math.pow(deltaTime, 2));

        io.setPosition(inputs.position.plus(positionChange));

        super.periodic();

        visualizer.setAngle(inputs.position);
        if (inputs.velocityError != null && inputs.velocityError.lte(tolerance)) {
            visualizer.setColor(Color.GREEN);
        } else {
            visualizer.setColor(Color.BLACK);
        }
    }
}
