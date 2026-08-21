package com.maartenpeels.fpv.control;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;

import org.junit.jupiter.api.Nested;
import org.junit.jupiter.api.Test;

class PilotInputUpdateTest {

    @Nested
    class Validation {

        @Test
        void rejectsAMissingHalfBecauseACallerNeedsBothAnswers() {
            assertThrows(
                    IllegalArgumentException.class,
                    () -> new PilotInputUpdate(null, LookTrack.UNSET));
            assertThrows(
                    IllegalArgumentException.class,
                    () -> new PilotInputUpdate(ControlInput.NEUTRAL, null));
        }
    }

    @Nested
    class Accessors {

        @Test
        void carriesTheSticksAndTheNextLookMemory() {
            LookTrack track = LookTrack.at(0.5, -0.25);
            PilotInputUpdate update = new PilotInputUpdate(ControlInput.NEUTRAL, track);

            assertEquals(ControlInput.NEUTRAL, update.input());
            assertEquals(track, update.track());
        }
    }
}
