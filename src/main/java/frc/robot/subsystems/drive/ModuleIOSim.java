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

package frc.robot.subsystems.drive;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;

import org.wpilib.math.controller.PIDController;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.system.DCMotor;
import org.wpilib.math.system.Models;
import org.wpilib.math.util.Units;
import org.wpilib.simulation.DCMotorSim;
import org.wpilib.system.Timer;

/**
 * Physics sim implementation of module IO. The sim models are configured using a set of module
 * constants from Phoenix. Simulation is always based on voltage control.
 */
public class ModuleIOSim implements ModuleIO {
    // TunerConstants doesn't support separate sim constants, so they are declared
    // locally
    private static final double DRIVE_KP = 0.05;
    private static final double DRIVE_KD = 0.0;
    private static final double DRIVE_KS = 0.0;
    // Updated from 0.91035 to reflect new gear ratio (6.0 vs 7.3636): 0.91035 * (6.0/7.3636) =
    // 0.742
    private static final double DRIVE_KV_ROT = 0.742;
    private static final double DRIVE_KV = 1.0 / Units.rotationsToRadians(1.0 / DRIVE_KV_ROT);
    private static final double TURN_KP = 12.5;
    private static final double TURN_KD = 0.0;
    private static final DCMotor DRIVE_GEARBOX = DCMotor.getKrakenX60Foc(1);
    private static final DCMotor TURN_GEARBOX = DCMotor.getKrakenX60Foc(1);

    private final DCMotorSim driveSim;
    private final DCMotorSim turnSim;

    private boolean driveClosedLoop = false;
    private boolean turnClosedLoop = false;
    private PIDController driveController = new PIDController(DRIVE_KP, 0, DRIVE_KD);
    private PIDController turnController = new PIDController(TURN_KP, 0, TURN_KD);
    private double driveFFVolts = 0.0;
    private double driveAppliedVolts = 0.0;
    private double turnAppliedVolts = 0.0;

    /**
     * Constructs a new ModuleIOSim instance.
     *
     * @param constants Module-specific constants for configuring the simulation
     */
    public ModuleIOSim(
            SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
                    constants) {
        // Create drive and turn sim models
        driveSim =
                new DCMotorSim(
                        Models.singleJointedArmFromPhysicalConstants(
                                DRIVE_GEARBOX,
                                constants.DriveInertia,
                                constants.DriveMotorGearRatio),
                        DRIVE_GEARBOX);
        turnSim =
                new DCMotorSim(
                        Models.singleJointedArmFromPhysicalConstants(
                                TURN_GEARBOX,
                                constants.SteerInertia,
                                constants.SteerMotorGearRatio),
                        TURN_GEARBOX);

        // Enable wrapping for turn PID
        turnController.enableContinuousInput(-Math.PI, Math.PI);
    }

    @Override
    public void updateInputs(ModuleIOInputs inputs) {
        // Run closed-loop control
        if (driveClosedLoop) {
            driveAppliedVolts =
                    driveFFVolts + driveController.calculate(driveSim.getAngularVelocity());
        } else {
            driveController.reset();
        }

        if (turnClosedLoop) {
            turnAppliedVolts = turnController.calculate(turnSim.getAngularPosition());
        } else {
            turnController.reset();
        }

        // Update simulation state
        driveSim.setInputVoltage(Math.clamp(driveAppliedVolts, -12.0, 12.0));
        turnSim.setInputVoltage(Math.clamp(turnAppliedVolts, -12.0, 12.0));
        driveSim.update(0.02);
        turnSim.update(0.02);

        // Update drive inputs
        inputs.driveConnected = true;
        inputs.drivePositionRad = driveSim.getAngularPosition();
        inputs.driveVelocityRadPerSec = driveSim.getAngularVelocity();
        inputs.driveAppliedVolts = driveAppliedVolts;
        inputs.driveCurrentAmps = Math.abs(driveSim.getCurrentDraw());

        // Update turn inputs
        inputs.turnConnected = true;
        inputs.turnEncoderConnected = true;
        inputs.turnAbsolutePosition = new Rotation2d(turnSim.getAngularPosition());
        inputs.turnPosition = new Rotation2d(turnSim.getAngularPosition());
        inputs.turnVelocityRadPerSec = turnSim.getAngularVelocity();
        inputs.turnAppliedVolts = turnAppliedVolts;
        inputs.turnCurrentAmps = Math.abs(turnSim.getCurrentDraw());

        // Update odometry inputs (50Hz because high-frequency odometry in sim doesn't matter)
        inputs.odometryTimestamps = new double[] {Timer.getTimestamp()};
        inputs.odometryDrivePositionsRad = new double[] {inputs.drivePositionRad};
        inputs.odometryTurnPositions = new Rotation2d[] {inputs.turnPosition};
    }

    @Override
    public void setDriveOpenLoop(double output) {
        driveClosedLoop = false;
        driveAppliedVolts = output;
    }

    @Override
    public void setTurnOpenLoop(double output) {
        turnClosedLoop = false;
        turnAppliedVolts = output;
    }

    @Override
    public void setDriveVelocity(double velocityRadPerSec) {
        driveClosedLoop = true;
        driveFFVolts = DRIVE_KS * Math.signum(velocityRadPerSec) + DRIVE_KV * velocityRadPerSec;
        driveController.setSetpoint(velocityRadPerSec);
    }

    @Override
    public void setTurnPosition(Rotation2d rotation) {
        turnClosedLoop = true;
        turnController.setSetpoint(rotation.getRadians());
    }
}
