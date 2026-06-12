// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/** Rewrite from scratch for practice only. JW */
package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;

import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.MomentOfInertia;

import lombok.AccessLevel;
import lombok.NoArgsConstructor;

/** docs */
@NoArgsConstructor(access = AccessLevel.PRIVATE)
public class FlywheelConstants {

    // Identity & physical/calibration constants
    public static final String NAME = "Flywheel";

    public static final AngularVelocity MAX_VELOCITY = RotationsPerSecond.of(20.0);
    public static final AngularAcceleration MAX_ACCELERATION = RotationsPerSecondPerSecond.of(60.0);
    public static final AngularVelocity TOLERANCE = RotationsPerSecond.of(1.0);

    private static final double GEARING = 1.0;
    public static final Distance WHEEL_RADIUS = Inches.of(3.0);
    public static final MomentOfInertia MOI = KilogramSquareMeters.of(0.0);

    // tbc

}
