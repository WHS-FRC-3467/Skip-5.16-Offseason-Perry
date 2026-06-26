// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/** Rewrite from scratch for practice only. JW */
package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.wpilibj.RobotBase;
import frc.lib.util.PID;
import lombok.AccessLevel;
import lombok.NoArgsConstructor;

/** docs */
@NoArgsConstructor(access = AccessLevel.PRIVATE)
public class FlywheelConstants {

    // Identity & physical/calibration constants
    public static final String NAME = "Flywheel";

    public static final AngularVelocity MAX_VELOCITY = RotationsPerSecond.of(69.0);
    public static final AngularAcceleration MAX_ACCELERATION = RotationsPerSecondPerSecond.of(30.0);
    public static final AngularAcceleration BROWNOUT_MAX_ACCELERATION = RotationsPerSecondPerSecond.of(15.0);
    public static final AngularVelocity TOLERANCE = RotationsPerSecond.of(1.0);
    
    // Sensor-to-mechanism gearing ratio (motor rotations per mechanism rotation)
    private static final double GEARING = (32.0 / 24.0);
    public static final Distance FLYWHEEL_RADIUS = Inches.of(4.0);
    public static final Mass FLYWHEEL_MASS = Kilograms.of(4.95);
    public static final MomentOfInertia MOI = KilogramSquareMeters.of(FLYWHEEL_MASS.in(Kilograms) * FLYWHEEL_RADIUS.in(Meters) * FLYWHEEL_RADIUS.in(Meters));
    private static final DCMotor DCMOTOR = DCMotor.getKrakenX60(4);

    private static final PID getPID() {
        if (RobotBase.isReal()) {
            return new PID(16.0, 0.0, 0.0).withS(5.5).withA(0.8);
        }
        else {
            return new PID(10.0, 0.0, 0.0).withV(1.8);
        }
    }

    // Speed PID 
    public static final PID SLOT0_PID = getPID();

    // tbc

}
