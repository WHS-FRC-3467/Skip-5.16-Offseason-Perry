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

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.wpilibj.RobotBase;

import frc.lib.io.motor.MotorIO;
import frc.lib.io.motor.MotorIO.PIDSlot;
import frc.lib.io.motor.MotorIOTalonFX;
import frc.lib.io.motor.MotorIOTalonFX.TalonFXFollower;
import frc.lib.io.motor.MotorIOTalonFXSim;
import frc.lib.mechanisms.flywheel.FlywheelMechanism;
import frc.lib.mechanisms.flywheel.FlywheelMechanismReal;
import frc.lib.mechanisms.flywheel.FlywheelMechanismSim;
import frc.lib.util.PID;
import frc.robot.Constants;
import frc.robot.Ports;
import frc.robot.Robot;

import lombok.AccessLevel;
import lombok.NoArgsConstructor;

/** Constants class describing the operating parameters of the flywheel mechanism. */
@NoArgsConstructor(access = AccessLevel.PRIVATE)
public class FlywheelConstants {

    // Identity & physical/calibration constants
    public static final String NAME = "Flywheel";

    public static final AngularVelocity MAX_VELOCITY = RotationsPerSecond.of(69.0);
    public static final AngularAcceleration MAX_ACCELERATION = RotationsPerSecondPerSecond.of(30.0);
    public static final AngularAcceleration BROWNOUT_MAX_ACCELERATION =
            RotationsPerSecondPerSecond.of(15.0);
    public static final AngularVelocity TOLERANCE = RotationsPerSecond.of(1.0);

    // Sensor-to-mechanism gearing ratio (motor rotations per mechanism rotation)
    private static final double GEARING = (32.0 / 24.0);
    public static final Distance FLYWHEEL_RADIUS = Inches.of(4.0);
    public static final Mass FLYWHEEL_MASS = Kilograms.of(4.95);
    // Approximated as thin-walled hoop I ~ M * R^2
    public static final MomentOfInertia MOI =
            KilogramSquareMeters.of(
                    FLYWHEEL_MASS.in(Kilograms)
                            * FLYWHEEL_RADIUS.in(Meters)
                            * FLYWHEEL_RADIUS.in(Meters));
    private static final DCMotor DCMOTOR = DCMotor.getKrakenX60(4);

    private static final PID getPID() {
        if (RobotBase.isReal()) {
            return new PID(16.0, 0.0, 0.0).withS(5.5).withA(0.8);
        } else {
            return new PID(10.0, 0.0, 0.0).withV(1.8);
        }
    }

    // Speed PID
    public static final PID SLOT0_PID = getPID();

    /**
     * Motor configuration for flywheel's TalonFX motor package. Configuration incldues
     * supply/torque current, applied voltage, position, and kinematic limits as well as neutral
     * mode behavior and gearing to allow for mechanism-space setpoints.
     */
    public static TalonFXConfiguration getFXConfig() {
        TalonFXConfiguration config = new TalonFXConfiguration();

        config.CurrentLimits.SupplyCurrentLimitEnable = false;
        config.CurrentLimits.SupplyCurrentLimit = 40.0;
        config.CurrentLimits.SupplyCurrentLowerLimit = 40.0;
        config.CurrentLimits.SupplyCurrentLowerTime = 0.1;

        if (Robot.isReal()) {
            config.TorqueCurrent.PeakForwardTorqueCurrent = 80.0;
            config.TorqueCurrent.PeakReverseTorqueCurrent = -80.0;
        }

        config.Voltage.PeakForwardVoltage = 12.0;
        config.Voltage.PeakReverseVoltage = -12.0;

        config.MotionMagic.MotionMagicCruiseVelocity = MAX_VELOCITY.in(RotationsPerSecond);
        config.MotionMagic.MotionMagicAcceleration =
                MAX_ACCELERATION.in(RotationsPerSecondPerSecond);

        config.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
        config.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;

        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        config.Feedback.SensorToMechanismRatio = GEARING;
        config.Feedback.RotorToSensorRatio = 1.0;

        config.Slot0 = Slot0Configs.from(SLOT0_PID.toSlotConfigs());

        return config;
    }

    /**
     * Creates and configures the flywheel mechanism based on the current robot mode. Selects the
     * appropriate implementation (real, sim, or replay) and enables tunable PID.
     *
     * @return configured flywheel mechanism
     */
    public static FlywheelMechanism<?> get() {
        FlywheelMechanism<?> flywheel;
        switch (Constants.currentMode) {
            case REAL:
                flywheel =
                        new FlywheelMechanismReal(
                                NAME,
                                new MotorIOTalonFX(
                                        NAME,
                                        getFXConfig(),
                                        Ports.topLeftFlywheel,
                                        new TalonFXFollower(Ports.topRightFlywheel, true),
                                        new TalonFXFollower(Ports.bottomLeftFlywheel, false),
                                        new TalonFXFollower(Ports.bottomRightFlywheel, true)));
                break;
            case SIM:
                flywheel =
                        new FlywheelMechanismSim(
                                NAME,
                                new MotorIOTalonFXSim(
                                        NAME,
                                        getFXConfig(),
                                        Ports.topLeftFlywheel,
                                        new TalonFXFollower(Ports.topRightFlywheel, true),
                                        new TalonFXFollower(Ports.bottomLeftFlywheel, false),
                                        new TalonFXFollower(Ports.bottomRightFlywheel, true)),
                                DCMOTOR,
                                MOI,
                                TOLERANCE);

                break;
            case REPLAY:
                flywheel =
                        new FlywheelMechanism<>(
                                NAME,
                                new MotorIO() {
                                    @Override
                                    public int getNumberOfMotors() {
                                        return 4;
                                    }
                                }) {};
                break;
            default:
                throw new IllegalStateException("Unrecognized Robot Mode");
        }

        flywheel.enableTunablePID(PIDSlot.SLOT_0, SLOT0_PID);
        flywheel.withRadius(FLYWHEEL_RADIUS);

        return flywheel;
    }
}
