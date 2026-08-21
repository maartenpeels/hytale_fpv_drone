package com.maartenpeels.fpv.control;

/**
 * {@link PilotInputMapper#map}'s two answers: the stick positions to fly with now, and the look
 * memory to carry into the next packet.
 */
public record PilotInputUpdate(ControlInput input, LookTrack track) {

    public PilotInputUpdate {
        if (input == null) {
            throw new IllegalArgumentException("input must not be null");
        }
        if (track == null) {
            throw new IllegalArgumentException("track must not be null");
        }
    }
}
