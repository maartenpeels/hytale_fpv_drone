package com.maartenpeels.fpv.flight;

/**
 * A normalised torque demand on each axis, {@code [-1, 1]} — what {@link RatePid} produces and
 * {@link MotorMixer} consumes.
 *
 * <p>Signs are {@link com.maartenpeels.fpv.control.ControlInput}'s, as everywhere else in the
 * flight model: positive roll banks right, positive pitch is nose <strong>down</strong>, positive
 * yaw is nose right.
 *
 * <p>Normalised means "a fraction of the differential thrust the mixer can represent", not a torque
 * in N·m and not an angular acceleration. {@code 1} is as hard as this axis can be pushed given the
 * collective it has to share the motors with; what that works out to in rad/s² depends on
 * {@link QuadParameters#rollPitchAuthority()} and on the throttle, which is why the conversion
 * happens in {@link QuadIntegrator} and not here.
 *
 * <p>A distinct type rather than three loose doubles because {@link BodyRates} is the same shape
 * with entirely different units, and passing one where the other belongs would compile silently.
 */
public record TorqueDemand(double roll, double pitch, double yaw) {

    public static final TorqueDemand NONE = new TorqueDemand(0, 0, 0);

    public TorqueDemand {
        requireInRange(roll, "roll");
        requireInRange(pitch, "pitch");
        requireInRange(yaw, "yaw");
    }

    private static void requireInRange(double value, String axis) {
        if (!Double.isFinite(value) || value < -1.0 || value > 1.0) {
            throw new IllegalArgumentException(
                    axis + " torque must be within [-1, 1] but was " + value);
        }
    }
}
