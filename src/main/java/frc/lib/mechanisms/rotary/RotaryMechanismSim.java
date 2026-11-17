// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.mechanisms.rotary;

import static org.wpilib.units.Units.KilogramSquareMeters;
import static org.wpilib.units.Units.Meters;
import static org.wpilib.units.Units.Radians;
import static org.wpilib.units.Units.RadiansPerSecond;
import static org.wpilib.units.Units.Seconds;
import static org.wpilib.units.Units.Volts;

import frc.lib.io.absoluteencoder.AbsoluteEncoderIOSim;
import frc.lib.io.motor.MotorIOSim;

import org.wpilib.math.system.DCMotor;
import org.wpilib.simulation.BatterySim;
import org.wpilib.simulation.RoboRioSim;
import org.wpilib.simulation.SingleJointedArmSim;
import org.wpilib.system.RobotController;
import org.wpilib.units.measure.MomentOfInertia;
import org.wpilib.units.measure.Time;

import java.util.Optional;

/**
 * A simulated implementation of the RotaryMechanism base class that uses SingleJointedArmSim to
 * simulate the behavior of a rotary mechanism.
 */
public class RotaryMechanismSim extends RotaryMechanism<MotorIOSim, AbsoluteEncoderIOSim> {
    private final SingleJointedArmSim sim;

    private Time lastTime = RobotController.getMeasureTime();

    public RotaryMechanismSim(
            String name,
            MotorIOSim io,
            DCMotor motor,
            MomentOfInertia momentOfInertia,
            Boolean useGravity,
            RotaryMechCharacteristics characteristics,
            Optional<AbsoluteEncoderIOSim> absoluteEncoder,
            String encoderName) {
        super(name, characteristics, io, absoluteEncoder, encoderName);

        if (momentOfInertia.isEquivalent(KilogramSquareMeters.zero()))
            throw new IllegalArgumentException("momentOfInertia must be greater than zero!");

        sim =
                new SingleJointedArmSim(
                        motor,
                        io.getRotorToSensorRatio() * io.getSensorToMechanismRatio(),
                        momentOfInertia.in(KilogramSquareMeters),
                        characteristics.armLength().in(Meters),
                        characteristics.minAngle().in(Radians),
                        characteristics.maxAngle().in(Radians),
                        useGravity,
                        characteristics.startingAngle().in(Radians));
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

        io.setPosition(Radians.of(sim.getAngle()));
        io.setRotorVelocity(
                RadiansPerSecond.of(sim.getVelocity())
                        .times(io.getRotorToSensorRatio() * io.getSensorToMechanismRatio()));

        absoluteEncoder.ifPresent(encoderSim -> {
            encoderSim.setAngle(Radians.of(sim.getAngle()).times(io.getSensorToMechanismRatio()));
        });

        super.periodic();
    }
}
