package com.maartenpeels.fpv.plugin.drone;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.hypixel.hytale.codec.EmptyExtraInfo;
import org.bson.BsonDocument;
import org.junit.jupiter.api.Nested;
import org.junit.jupiter.api.Test;

/**
 * Round-trips {@link ParkedBody#CODEC}.
 *
 * <p>This is the one codec in the feature that carries a decision rather than a value, and the
 * whole crash-recovery design rests on it: if the two field bindings were transposed, a Creative
 * pilot who crashed mid-flight would come back with the wrong marker stripped — which is the exact
 * bug {@link ParkedBody} exists to prevent — and every ECS test would still pass, because they all
 * construct the component directly and never serialize it.
 */
class ParkedBodyTest {

    @Nested
    class SurvivesSerialisation {

        @Test
        void keepsTheTwoFlagsDistinctSoTheWrongMarkerIsNeverStripped() {
            // Asymmetric on purpose: a transposed binding is invisible to (true, true).
            ParkedBody decoded = roundTrip(new ParkedBody(true, false));

            assertTrue(decoded.addedInvulnerable(), "AddedInvulnerable must map back to addedInvulnerable");
            assertFalse(decoded.addedIntangible(), "AddedIntangible must map back to addedIntangible");
        }

        @Test
        void keepsTheOppositeAssignmentDistinctToo() {
            ParkedBody decoded = roundTrip(new ParkedBody(false, true));

            assertFalse(decoded.addedInvulnerable());
            assertTrue(decoded.addedIntangible());
        }

        @Test
        void namesTheFieldsAsTheSaveFileKeysTheyAre() {
            // These strings are on-disk keys. Renaming one silently orphans every parked body
            // already saved, so pin them.
            BsonDocument encoded = ParkedBody.CODEC.encode(new ParkedBody(true, true), EmptyExtraInfo.EMPTY);

            assertTrue(encoded.containsKey("AddedInvulnerable"), encoded.toJson());
            assertTrue(encoded.containsKey("AddedIntangible"), encoded.toJson());
        }

        @Test
        void hasAStableComponentIdBecauseItIsASaveFileKey() {
            assertEquals("FpvParkedBody", ParkedBody.ID);
        }
    }

    @Nested
    class Cloning {

        @Test
        void copiesBothFlagsBecauseAWorldSwitchClonesTheComponent() {
            ParkedBody clone = (ParkedBody) new ParkedBody(true, false).clone();

            assertTrue(clone.addedInvulnerable());
            assertFalse(clone.addedIntangible());
        }
    }

    private static ParkedBody roundTrip(ParkedBody original) {
        BsonDocument encoded = ParkedBody.CODEC.encode(original, EmptyExtraInfo.EMPTY);
        return ParkedBody.CODEC.decode(encoded, EmptyExtraInfo.EMPTY);
    }
}
