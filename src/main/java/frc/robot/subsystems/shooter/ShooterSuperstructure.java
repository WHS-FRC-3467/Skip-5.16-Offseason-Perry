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

/** Rewrite from scratch for practice only. JW */
package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;
import static edu.wpi.first.units.Units.Watts;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Power;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.lib.io.motor.MotorIO.PIDSlot;
import frc.lib.mechanisms.flywheel.FlywheelMechanism;
import frc.lib.mechanisms.rotary.RotaryMechanism;
import frc.lib.util.LoggedTrigger;
import frc.lib.util.LoggedTunableNumber;
import frc.robot.RobotState;
import frc.robot.RobotState.Target;
import frc.robot.util.ShotTracker;

import java.util.function.Supplier;

public class ShooterSuperstructure extends SubsystemBase implements AutoCloseable {

    // Dependencies
    private final RobotState robotState = RobotState.getInstance();
    private final FlywheelMechanism<?> flywheelIO;
    private final RotaryMechanism<?, ?> hoodIO;

    // Shot model / calibration / associated trackers
    /** Distance from hub in meters -> flywheel speed in rotations per second */
    private static final InterpolatingDoubleTreeMap hubFlywheelMap =
            new InterpolatingDoubleTreeMap();

    static {
        hubFlywheelMap.put(1.8, 26.5);
        hubFlywheelMap.put(2.1, 26.5);
        hubFlywheelMap.put(2.5, 28.0);
        hubFlywheelMap.put(3.15, 32.0);
        hubFlywheelMap.put(3.55, 32.0);
        hubFlywheelMap.put(4.0, 34.0);
        hubFlywheelMap.put(4.5, 34.0);
        hubFlywheelMap.put(5.0, 36.0);
    }

    /** Distance from feed pose in meters -> flywheel speed in rotations per second */
    private static final InterpolatingDoubleTreeMap feedFlywheelMap =
            new InterpolatingDoubleTreeMap();

    static {
        feedFlywheelMap.put(3.35, 25.0);
        feedFlywheelMap.put(4.5, 32.0);
        feedFlywheelMap.put(5.5, 35.0);
        feedFlywheelMap.put(8.0, 37.0);
    }

    // Distance from hub in meters -> hood angle in degrees
    private static final InterpolatingDoubleTreeMap hubHoodMap = new InterpolatingDoubleTreeMap();

    static {
        hubHoodMap.put(1.8, 0.0);
        hubHoodMap.put(2.1, 6.0);
        hubHoodMap.put(2.51, 7.0);
        hubHoodMap.put(3.15, 8.67);
        hubHoodMap.put(3.55, 10.0);
        hubHoodMap.put(4.0, 13.0);
        hubHoodMap.put(4.5, 16.0);
        hubHoodMap.put(5.0, 21.0);
    }

    // Distance from feed pose in meters -> hood angle in degrees
    private static final InterpolatingDoubleTreeMap feedHoodMap = new InterpolatingDoubleTreeMap();

    static {
        feedHoodMap.put(4.5, 24.0);
        feedHoodMap.put(5.0, 24.0);
        feedHoodMap.put(5.5, 27.0);
        feedHoodMap.put(7.0, 27.0);
        feedHoodMap.put(8.0, 27.0);
        feedHoodMap.put(9.0, 27.0);
        feedHoodMap.put(10.0, 27.0);
    }

    private final ShotTracker shotTracker;
    private final LoggedTunableNumber flywheelProfileDebounceSeconds =
            new LoggedTunableNumber(getName() + "/FlywheelProfileDebounceSeconds", 0.04);
    private final LoggedTunableNumber nearGoalDebounceSeconds =
            new LoggedTunableNumber(getName() + "/NearGoalDebounceSeconds", 0.2);
    private final LoggedTunableNumber flywheelProfileToleranceRPS =
            new LoggedTunableNumber(getName() + "/FlywheelProfileToleranceRPS", 0.2);

    // Public status signals & helpers
    /** Trigger determining if flywheel motion profile is complete. */
    public final LoggedTrigger flywheelProfileComplete =
            new LoggedTrigger(
                            getName() + "/flywheelProfileComplete",
                            () -> isFlywheelProfileComplete())
                    .debounce(flywheelProfileDebounceSeconds.get());

    /**
     * Trigger determining whether flywheel is at speed and hood is at angle appropriate for the
     * current distance from target (feed or shot).
     */
    public final LoggedTrigger shooterAtDesiredState =
            new LoggedTrigger(
                    getName() + "/shooterAtGoal", () -> isFlywheelAtSpeed() && isHoodAtAngle());

    /**
     * Trigger determining whether robot is ready to shoot at its current target (i.e. {@link
     * #isNearGoal} is true and the robot is aligned with current target.
     */
    public final LoggedTrigger readyToShootAtCurrentTarget =
            new LoggedTrigger(
                    getName() + "/readyToShootAtCurrentTarget",
                    () -> {
                        if (robotState.shouldFeed.getAsBoolean()) {
                            return isNearGoal() && robotState.facingFeedTarget.getAsBoolean();
                        } else {
                            return isNearGoal() && robotState.facingTarget.getAsBoolean();
                        }
                    });

    /**
     * Trigger determining whether current flywheel velocity is near motion profile goal velocity.
     */
    public final LoggedTrigger nearGoal =
            new LoggedTrigger(getName() + "/nearGoal", () -> isNearGoal())
                    .debounce(nearGoalDebounceSeconds.get());

    /**
     * Trigger determining whether the robot is in a static shooting state; does not trigger during
     * static feeding.
     */
    public final LoggedTrigger staticShotState =
            new LoggedTrigger(
                    getName() + "/staticShotState",
                    () ->
                            isNearGoal()
                                    && robotState.atStaticShootingPosition.getAsBoolean()
                                    && robotState.getTarget() == Target.HUB);

    // Constructor
    public ShooterSuperstructure(RotaryMechanism<?, ?> hoodIO, FlywheelMechanism<?> flywheelIO) {
        this.flywheelIO = flywheelIO;
        this.hoodIO = hoodIO;
        shotTracker = ShotTracker.create(this);
    }

    // Periodic
    @Override
    public void periodic() {
        // tuning updates
        // IO updates
        // trigger polling
        // logging
    }

    // Goal computation helpers
    private void getFlywheelTrim() {}

    private AngularVelocity getFlywheelTrimStep() {
        return RotationsPerSecond.of(0);
    }

    // Actuator helpers
    private void applyFlywheelVelocity(AngularVelocity velocity) {
        flywheelIO.runVelocity(velocity, FlywheelConstants.MAX_ACCELERATION, PIDSlot.SLOT_0);
    }

    private void applyHoodPosition(Angle angle) {
        hoodIO.runUnprofiledPosition(angle, PIDSlot.SLOT_0);
    }

    // State helpers
    private boolean isFlywheelProfileComplete() {
        return flywheelIO
                .getVelocitySetpoint()
                .isNear(
                        flywheelIO.getVelocityGoal(),
                        RotationsPerSecond.of(flywheelProfileToleranceRPS.get()));
    }

    private boolean isNearGoal() {
        return flywheelIO
                .getVelocity()
                .isNear(
                        flywheelIO.getVelocityGoal(),
                        RotationsPerSecond.of(flywheelProfileToleranceRPS.get()));
    }

    private boolean isFlywheelAtSpeed() {
        return flywheelIO
                .getVelocity()
                .isNear(getDesiredFlywheelVelocity(), FlywheelConstants.TOLERANCE);
    }

    private boolean isHoodAtAngle() {
        return hoodIO.getPosition().isNear(getDesiredHoodAngle(), HoodConstants.TOLERANCE);
    }

    // Accessors
    /** Return the current flywheel angular velocity (mechanism-space) */
    public AngularVelocity getFlywheelVelocity() {
        return flywheelIO.getVelocity();
    }

    /** Return the current flywheel linear velocity (mechanism-space) */
    public LinearVelocity getFlywheelLinearVelocity() {
        return MetersPerSecond.of(
                getFlywheelVelocity().in(RadiansPerSecond)
                        * FlywheelConstants.FLYWHEEL_RADIUS.in(Meters));
    }

    /** Return the desired flywheel linear velocity (mechanism-space) */
    public LinearVelocity getDesiredFlywheelLinearVelocity() {
        return MetersPerSecond.of(
                getDesiredFlywheelVelocity().in(RadiansPerSecond)
                        * FlywheelConstants.FLYWHEEL_RADIUS.in(Meters));
    }

    /** Return the current angle of the hood (mechanism-space) */
    public Angle getHoodAngle() {
        return hoodIO.getPosition();
    }

    /** Return the total power draw of the flywheel */
    public Power getFlywheelPowerDraw() {
        return Watts.of(
                flywheelIO.getSupplyCurrent().in(Amps) * flywheelIO.getAppliedVoltage().in(Volts));
    }

    private AngularVelocity getDesiredFlywheelVelocity() {
        double distanceMeters = getShooterDistance(robotState.shouldFeed.getAsBoolean());
        if (robotState.shouldFeed.getAsBoolean()) {
            return RotationsPerSecond.of(feedFlywheelMap.get(distanceMeters));
        } else {
            return RotationsPerSecond.of(hubFlywheelMap.get(distanceMeters));
        }
    }

    private Angle getDesiredHoodAngle() {
        double distanceMeters = getShooterDistance(robotState.shouldFeed.getAsBoolean());
        if (robotState.shouldFeed.getAsBoolean()) {
            return Degrees.of(feedHoodMap.get(distanceMeters));
        } else {
            return Degrees.of(hubHoodMap.get(distanceMeters));
        }
    }

    private double getShooterDistance(boolean shouldFeed) {
        if (shouldFeed) {
            return robotState
                    .getFuturePose(robotState.feedLookaheadSeconds.get())
                    .getTranslation()
                    .getDistance(robotState.getTarget().getAllianceTranslation().toTranslation2d());
        } else {
            return robotState.getDistanceToTarget().in(Meters);
        }
    }

    // Command factory
    public Command setFlywheelSpeed(AngularVelocity velocity) {
        return this.runOnce(() -> applyFlywheelVelocity(velocity));
    }

    public Command setHoodAngle(Angle angle) {
        return this.runOnce(() -> applyHoodPosition(angle));
    }

    public Command spinUpFlywheel() {
        return Commands.none();
    }

    public Command setShooterToFixedDistance(Distance distance, boolean isFeeding) {
        return Commands.none();
    }

    public Command setShooterContinuous() {
        return Commands.none();
    }

    public Command fountain() {
        return Commands.none();
    }

    public Command coastFlywheels() {
        return Commands.none();
    }

    public Command stopAndStow() {
        return Commands.none();
    }

    public Command retractHood() {
        return Commands.none();
    }

    public Command homeHood() {
        return Commands.none();
    }

    public Command trimFlywheelSpeedUp() {
        return Commands.none();
    }

    public Command trimFlywheelSpeedDown() {
        return Commands.none();
    }

    private Command setShooterCommand(
            Supplier<AngularVelocity> flywheelVelocity, Supplier<Angle> hoodAngle, String name) {
        return Commands.none();
    }

    @Override
    public void close() {}
}
