package com.maartenpeels.fpv.flight;

/**
 * The airframe's physical characteristics — everything {@link QuadIntegrator} needs that is not
 * state or input.
 *
 * <p>Units are Hytale world units, seconds and radians. {@code gravity} defaults to {@code 32.0}
 * rather than {@code 9.81} because that is the server's own figure
 * ({@code PhysicsConstants.GRAVITY_ACCELERATION}); Hytale's world is not metric-consistent, and a
 * drone built around real gravity would feel like it was flying on the moon next to everything else
 * in it.
 *
 * <p>This is the <em>airframe</em>, which is server-wide. The pilot's rate-loop tune is
 * {@link RatePidGains}, persisted per player by #5, and deliberately not part of this record.
 *
 * <p>Build these with {@link #builder()} rather than the canonical constructor — eight positional
 * doubles is a transposition waiting to happen. {@link #DEFAULT} is a plausible racing quad; the
 * numbers are informed guesses about *feel*, and #24 is where they get judged.
 */
public record QuadParameters(
        double gravity,
        double thrustToWeight,
        double linearDrag,
        double quadraticDrag,
        double angularDrag,
        BodyRates maxRates,
        double rollPitchAuthority,
        double yawAuthority) {

    public static final double DEFAULT_GRAVITY = 32.0;

    /** Mid-range for a racing build. Freestyle quads sit nearer 6, racers past 10. */
    public static final double DEFAULT_THRUST_TO_WEIGHT = 8.0;

    /** Gentle low-speed bleed-off, per second. */
    public static final double DEFAULT_LINEAR_DRAG = 0.1;

    /** Per world unit. With default gravity this puts free-fall terminal velocity near 57 u/s. */
    public static final double DEFAULT_QUADRATIC_DRAG = 0.01;

    /** Passive damping of body rotation, per second. */
    public static final double DEFAULT_ANGULAR_DRAG = 1.0;

    /**
     * Angular acceleration at full roll or pitch differential thrust, rad/s². Referenced by
     * {@link RatePidGains#DEFAULT}, whose gains are only meaningful against a known authority.
     */
    public static final double DEFAULT_ROLL_PITCH_AUTHORITY = Math.toRadians(10_000.0);

    /** The same for yaw, which on a real quad comes from prop reaction and is far weaker. */
    public static final double DEFAULT_YAW_AUTHORITY = Math.toRadians(2_500.0);

    /** Full-stick rates: 800 °/s on roll and pitch, 400 °/s on yaw. */
    public static final BodyRates DEFAULT_MAX_RATES =
            new BodyRates(Math.toRadians(800.0), Math.toRadians(800.0), Math.toRadians(400.0));

    public QuadParameters {
        requirePositive(gravity, "gravity");
        requirePositive(thrustToWeight, "thrustToWeight");
        requireNonNegative(linearDrag, "linearDrag");
        requireNonNegative(quadraticDrag, "quadraticDrag");
        requireNonNegative(angularDrag, "angularDrag");
        requirePositive(rollPitchAuthority, "rollPitchAuthority");
        requirePositive(yawAuthority, "yawAuthority");
        if (maxRates == null || !maxRates.isFinite()) {
            throw new IllegalArgumentException("maxRates must be finite but was " + maxRates);
        }
        if (maxRates.roll() <= 0 || maxRates.pitch() <= 0 || maxRates.yaw() <= 0) {
            throw new IllegalArgumentException("every max rate must be positive but was " + maxRates);
        }
    }

    public static Builder builder() {
        return new Builder();
    }

    public static final QuadParameters DEFAULT = builder().build();

    /** Acceleration produced with all four motors at full command, in world units per second². */
    public double maxThrustAcceleration() {
        return this.thrustToWeight * this.gravity;
    }

    /**
     * The uniform motor command that exactly cancels gravity while level.
     *
     * <p>The square root is not a fudge: motor thrust goes with the square of the command, so hover
     * lands near a third of stick instead of at {@code 1/thrustToWeight}. That is where a real quad
     * hovers, and it is the single most noticeable throttle-feel detail in the model.
     *
     * <p>Returns more than {@code 1} for an airframe whose thrust cannot beat its own weight, which
     * is honest rather than useful — such a quad simply cannot hover, and clamping would hide that.
     * Callers turning this into a {@code ControlInput} throttle need to handle the case.
     */
    public double hoverCollective() {
        return Math.sqrt(1.0 / this.thrustToWeight);
    }

    private static void requirePositive(double value, String name) {
        if (!Double.isFinite(value) || value <= 0.0) {
            throw new IllegalArgumentException(name + " must be finite and positive but was " + value);
        }
    }

    private static void requireNonNegative(double value, String name) {
        if (!Double.isFinite(value) || value < 0.0) {
            throw new IllegalArgumentException(
                    name + " must be finite and non-negative but was " + value);
        }
    }

    /** Mutable builder over {@link #DEFAULT}'s values; every setter is optional. */
    public static final class Builder {

        private double gravity = DEFAULT_GRAVITY;
        private double thrustToWeight = DEFAULT_THRUST_TO_WEIGHT;
        private double linearDrag = DEFAULT_LINEAR_DRAG;
        private double quadraticDrag = DEFAULT_QUADRATIC_DRAG;
        private double angularDrag = DEFAULT_ANGULAR_DRAG;
        private BodyRates maxRates = DEFAULT_MAX_RATES;
        private double rollPitchAuthority = DEFAULT_ROLL_PITCH_AUTHORITY;
        private double yawAuthority = DEFAULT_YAW_AUTHORITY;

        private Builder() {}

        public Builder gravity(double gravity) {
            this.gravity = gravity;
            return this;
        }

        public Builder thrustToWeight(double thrustToWeight) {
            this.thrustToWeight = thrustToWeight;
            return this;
        }

        public Builder linearDrag(double linearDrag) {
            this.linearDrag = linearDrag;
            return this;
        }

        public Builder quadraticDrag(double quadraticDrag) {
            this.quadraticDrag = quadraticDrag;
            return this;
        }

        public Builder angularDrag(double angularDrag) {
            this.angularDrag = angularDrag;
            return this;
        }

        /** Zeroes every drag term, for tests that want a closed-form answer to compare against. */
        public Builder withoutDrag() {
            return this.linearDrag(0.0).quadraticDrag(0.0).angularDrag(0.0);
        }

        public Builder maxRates(BodyRates maxRates) {
            this.maxRates = maxRates;
            return this;
        }

        public Builder rollPitchAuthority(double rollPitchAuthority) {
            this.rollPitchAuthority = rollPitchAuthority;
            return this;
        }

        public Builder yawAuthority(double yawAuthority) {
            this.yawAuthority = yawAuthority;
            return this;
        }

        public QuadParameters build() {
            return new QuadParameters(
                    this.gravity,
                    this.thrustToWeight,
                    this.linearDrag,
                    this.quadraticDrag,
                    this.angularDrag,
                    this.maxRates,
                    this.rollPitchAuthority,
                    this.yawAuthority);
        }
    }
}
