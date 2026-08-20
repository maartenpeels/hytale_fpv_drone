package com.maartenpeels.fpv.flight;

/**
 * The four motor commands of an X-frame quad, each {@code 0..1} from idle to full.
 *
 * <p>Named rather than indexed on purpose. Flight-controller firmwares disagree about motor
 * numbering — Betaflight's motor 1 is the rear-right, others start front-left — and an off-by-one
 * in a mixer is invisible in a unit test that only checks magnitudes, then shows up as a drone
 * that rolls when you ask it to pitch.
 *
 * <p>Spin directions, viewed from above: {@code frontLeft} and {@code rearRight} turn
 * counter-clockwise, {@code frontRight} and {@code rearLeft} clockwise. The airframe feels the
 * reaction torque, which is opposite, so raising the counter-clockwise pair yaws the drone right.
 */
public record MotorOutputs(double frontLeft, double frontRight, double rearLeft, double rearRight) {

    public static final MotorOutputs IDLE = new MotorOutputs(0, 0, 0, 0);

    /**
     * Normalised thrust produced by each motor, as the square of its command.
     *
     * <p>Rotor thrust goes with the square of angular velocity, and modelling that rather than a
     * linear command is what puts hover at roughly a third of stick — where a real quad hovers —
     * instead of at the reciprocal of the thrust-to-weight ratio.
     */
    public MotorOutputs thrusts() {
        return new MotorOutputs(
                this.frontLeft * this.frontLeft,
                this.frontRight * this.frontRight,
                this.rearLeft * this.rearLeft,
                this.rearRight * this.rearRight);
    }

    /** Mean of the four values; applied to {@link #thrusts()} this is the collective fraction. */
    public double mean() {
        return (this.frontLeft + this.frontRight + this.rearLeft + this.rearRight) * 0.25;
    }

    /**
     * Left-minus-right half-difference. Positive means the left pair pushes harder, which drops the
     * right side and so banks right — matching positive {@code roll}.
     */
    public double rollDifferential() {
        return ((this.frontLeft + this.rearLeft) - (this.frontRight + this.rearRight)) * 0.5;
    }

    /**
     * Front-minus-rear half-difference. Positive means the front pair pushes harder, which drops
     * the nose — matching positive {@code pitch}.
     */
    public double pitchDifferential() {
        return ((this.frontLeft + this.frontRight) - (this.rearLeft + this.rearRight)) * 0.5;
    }

    /**
     * Half-difference between the counter-clockwise and clockwise diagonals. Positive means the
     * counter-clockwise pair pushes harder, and the opposing reaction torque yaws the nose right —
     * matching positive {@code yaw}.
     */
    public double yawDifferential() {
        return ((this.frontLeft + this.rearRight) - (this.frontRight + this.rearLeft)) * 0.5;
    }
}
