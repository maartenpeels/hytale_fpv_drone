package com.maartenpeels.fpv.plugin.camera;

import com.hypixel.hytale.protocol.RotationType;
import org.junit.jupiter.api.Nested;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNull;
import static org.junit.jupiter.api.Assertions.assertSame;
import static org.junit.jupiter.api.Assertions.assertTrue;

class DroneCameraViewTest {

    @Nested
    class RotationTypeMapping {

        @Test
        void trackedIsOffsetModeAndDrivenIsCustomBecauseThoseAreTheTwoThingsBeingBisected() {
            // If these two ever collapse onto the same RotationType, /fpv camera set stops being
            // able to tell "the client ignores roll" from "the client ignores EntityId".
            assertEquals(RotationType.AttachedToPlusOffset, DroneCameraView.TRACKED.rotationType());
            assertEquals(RotationType.Custom, DroneCameraView.DRIVEN.rotationType());
        }

        @Test
        void coversBothRotationTypesTheProtocolDefinesAndNoMore() {
            // RotationType has exactly two constants (protocol/RotationType.java:6-7), so the two
            // views are exhaustive over the protocol rather than an arbitrary pair.
            assertEquals(RotationType.values().length, DroneCameraView.values().length);
        }
    }

    @Nested
    class PerTickPush {

        @Test
        void onlyDrivenNeedsAPerTickPushBecauseOnlyItSendsAnAbsolutePosition() {
            // TRACKED lets the client follow the entity, so re-sending would be pure waste.
            assertTrue(DroneCameraView.DRIVEN.needsPerTickPush());
            assertFalse(DroneCameraView.TRACKED.needsPerTickPush());
        }
    }

    @Nested
    class Parsing {

        @Test
        void isCaseAndWhitespaceInsensitiveBecauseAHumanEditsTheConfigFile() {
            assertSame(DroneCameraView.DRIVEN, DroneCameraView.parseOrNull("driven"));
            assertSame(DroneCameraView.DRIVEN, DroneCameraView.parseOrNull("DRIVEN"));
            assertSame(DroneCameraView.DRIVEN, DroneCameraView.parseOrNull("  Driven  "));
            assertSame(DroneCameraView.TRACKED, DroneCameraView.parseOrNull("tracked"));
        }

        @Test
        void fallsBackRatherThanThrowingSoACameraTypoCannotStopThePluginLoading() {
            assertSame(DroneCameraView.TRACKED, DroneCameraView.parse("nonsense", DroneCameraView.TRACKED));
            assertSame(DroneCameraView.DRIVEN, DroneCameraView.parse(null, DroneCameraView.DRIVEN));
            assertSame(DroneCameraView.DRIVEN, DroneCameraView.parse("", DroneCameraView.DRIVEN));
        }

        @Test
        void parseOrNullRejectsUnknownValuesSoACommandCanReportTheTypoInstead() {
            // The strict counterpart matters: /fpv camera set exists to switch modes, so silently
            // staying in the old mode is the one outcome worse than an error message.
            assertNull(DroneCameraView.parseOrNull("nonsense"));
            assertNull(DroneCameraView.parseOrNull(""));
            assertNull(DroneCameraView.parseOrNull(null));
        }
    }
}
