package com.maartenpeels.fpv.flight;

/**
 * The normalised thrust each of the four motors is producing, {@code 0..1} — the *force* side of
 * {@link MotorOutputs}'s commands, obtained via {@link MotorOutputs#thrusts()}.
 *
 * <p>Kept distinct from {@code MotorOutputs} because the two are not interchangeable even though
 * both are four numbers in the same range. Torque is proportional to a difference in thrust, not in
 * command, so the differentials below are only meaningful here; and having a separate type means a
 * second {@code thrusts()} call cannot compile, so a command can never be squared twice.
 */
public record MotorThrusts(double frontLeft, double frontRight, double rearLeft, double rearRight) {

    public static final MotorThrusts NONE = new MotorThrusts(0, 0, 0, 0);

    /** Mean thrust — the fraction of the airframe's maximum being produced along the body's up axis. */
    public double collective() {
        return (this.frontLeft + this.frontRight + this.rearLeft + this.rearRight) * 0.25;
    }

    /**
     * Left-minus-right half-difference, in {@code [-1, 1]}. Positive means the left pair pushes
     * harder, which drops the right side and so banks right — matching positive {@code roll}.
     */
    public double rollDifferential() {
        return ((this.frontLeft + this.rearLeft) - (this.frontRight + this.rearRight)) * 0.5;
    }

    /**
     * Front-minus-rear half-difference, in {@code [-1, 1]}. Positive means the front pair pushes
     * harder, which drops the nose — matching positive {@code pitch}.
     */
    public double pitchDifferential() {
        return ((this.frontLeft + this.frontRight) - (this.rearLeft + this.rearRight)) * 0.5;
    }

    /**
     * Half-difference between the counter-clockwise and clockwise diagonals, in {@code [-1, 1]}.
     * Positive means the counter-clockwise pair pushes harder, and the opposing reaction torque yaws
     * the nose right — matching positive {@code yaw}.
     */
    public double yawDifferential() {
        return ((this.frontLeft + this.rearRight) - (this.frontRight + this.rearLeft)) * 0.5;
    }
}
