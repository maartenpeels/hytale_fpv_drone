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
 *
 * <p>A command is not a force. Force lives on {@link MotorThrusts}, which this converts to — the two
 * are separate types so that squaring twice, or reading a torque differential off raw commands,
 * cannot compile.
 */
public record MotorOutputs(double frontLeft, double frontRight, double rearLeft, double rearRight) {

    public static final MotorOutputs IDLE = new MotorOutputs(0, 0, 0, 0);

    /**
     * The thrust each motor actually produces, as the square of its command.
     *
     * <p>Rotor thrust goes with the square of angular velocity, and modelling that rather than a
     * linear command is what puts hover at roughly a third of stick — where a real quad hovers —
     * instead of at the reciprocal of the thrust-to-weight ratio.
     */
    public MotorThrusts thrusts() {
        return new MotorThrusts(
                this.frontLeft * this.frontLeft,
                this.frontRight * this.frontRight,
                this.rearLeft * this.rearLeft,
                this.rearRight * this.rearRight);
    }
}
