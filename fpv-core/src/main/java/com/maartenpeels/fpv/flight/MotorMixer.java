package com.maartenpeels.fpv.flight;

/**
 * Turns a collective throttle plus a normalised torque demand on each axis into four motor
 * commands.
 *
 * <p>Mixing is linear in command space, the way a flight controller does it:
 *
 * <pre>
 *   frontLeft  = collective + roll + pitch + yaw
 *   frontRight = collective - roll + pitch - yaw
 *   rearLeft   = collective + roll - pitch - yaw
 *   rearRight  = collective - roll - pitch + yaw
 * </pre>
 *
 * <p>Torque demands are in {@link com.maartenpeels.fpv.control.ControlInput}'s sign convention:
 * positive roll banks right, positive pitch is nose down, positive yaw is nose right.
 *
 * <p>The mix routinely asks for commands outside {@code [0, 1]} — full roll at full throttle wants
 * two motors above maximum. Clipping each motor independently would silently change the *ratio*
 * between them, turning a pure roll demand into roll plus an unwanted pitch. Two steps avoid that:
 *
 * <ol>
 *   <li>If the torque terms alone span more than the full command range, all three are scaled down
 *       together — never one at a time, so their ratios survive exactly.
 *   <li>The whole set is then shifted so it fits, moving the collective away from what the pilot
 *       asked for only as far as it has to.
 * </ol>
 *
 * <p>So it is the <em>collective</em> that yields to attitude, not the reverse: a roll demand at
 * full throttle pulls the throttle down rather than losing the roll, and a roll demand at closed
 * throttle lifts it slightly rather than leaving the drone with no authority at all. That is what
 * Betaflight's air mode does, and it is the behaviour a pilot expects.
 */
public final class MotorMixer {

    private MotorMixer() {}

    /**
     * @param collective total thrust command, {@code 0..1}
     * @param rollTorque normalised roll demand, {@code -1..1}
     * @param pitchTorque normalised pitch demand, {@code -1..1}
     * @param yawTorque normalised yaw demand, {@code -1..1}
     * @return four commands, each guaranteed within {@code [0, 1]}
     */
    public static MotorOutputs mix(
            double collective, double rollTorque, double pitchTorque, double yawTorque) {
        double frontLeft = rollTorque + pitchTorque + yawTorque;
        double frontRight = -rollTorque + pitchTorque - yawTorque;
        double rearLeft = rollTorque - pitchTorque - yawTorque;
        double rearRight = -rollTorque - pitchTorque + yawTorque;

        double spread =
                max(frontLeft, frontRight, rearLeft, rearRight)
                        - min(frontLeft, frontRight, rearLeft, rearRight);
        if (spread > 1.0) {
            // The mix is linear in the torque demands, so scaling the four results is the same as
            // scaling roll, pitch and yaw together -- and keeps their ratios exact.
            double scale = 1.0 / spread;
            frontLeft *= scale;
            frontRight *= scale;
            rearLeft *= scale;
            rearRight *= scale;
        }

        double lowest = min(frontLeft, frontRight, rearLeft, rearRight);
        double highest = max(frontLeft, frontRight, rearLeft, rearRight);
        double minimumOffset = -lowest;
        double maximumOffset = 1.0 - highest;
        // The torque terms now span at most 1.0, so some offset always fits them inside [0, 1] and
        // the collective is only moved as far as it must be. The inverted case is unreachable
        // except through rounding at exactly full spread; centring is the harmless answer.
        double offset =
                maximumOffset < minimumOffset
                        ? (minimumOffset + maximumOffset) * 0.5
                        : Math.clamp(collective, minimumOffset, maximumOffset);

        return new MotorOutputs(
                clampToRange(frontLeft + offset),
                clampToRange(frontRight + offset),
                clampToRange(rearLeft + offset),
                clampToRange(rearRight + offset));
    }

    private static double clampToRange(double command) {
        return Math.clamp(command, 0.0, 1.0);
    }

    private static double min(double a, double b, double c, double d) {
        return Math.min(Math.min(a, b), Math.min(c, d));
    }

    private static double max(double a, double b, double c, double d) {
        return Math.max(Math.max(a, b), Math.max(c, d));
    }
}
