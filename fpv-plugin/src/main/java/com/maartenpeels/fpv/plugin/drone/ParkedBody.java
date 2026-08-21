package com.maartenpeels.fpv.plugin.drone;

import com.hypixel.hytale.codec.Codec;
import com.hypixel.hytale.codec.KeyedCodec;
import com.hypixel.hytale.codec.builder.BuilderCodec;
import com.hypixel.hytale.component.Component;
import com.hypixel.hytale.server.core.universe.world.storage.EntityStore;

import javax.annotation.Nonnull;

/**
 * Records that we parked a pilot's character, and exactly which markers we added to do it.
 *
 * <p>Present means "this body is parked": {@code DroneLifecycleSystems.HideParkedBody} keys on
 * this component to hide the body from other clients, so presence and hidden-ness cannot drift
 * apart.
 *
 * <p><strong>Deliberately serialized</strong> — the one persisted component in this feature, and
 * the reason is worth reading. {@code Invulnerable} and {@code Intangible} are themselves
 * serialized: {@code EntityModule.java:328,330} registers both with an id and a codec. So a
 * server that dies mid-flight would bring the pilot back permanently invulnerable — an orphan
 * every bit as real as a stray drone entity. Because this component is saved and loaded by the
 * same mechanism as the markers it describes, either both survive or neither does, and the pair
 * can never disagree.
 *
 * <p>It records which markers <em>we</em> added rather than assuming, because the pilot may
 * already have had them: Creative mode adds {@code Invulnerable}
 * (`server/core/entity/entities/Player.java:752`). Clearing that on landing would be a bug.
 *
 * <p>Restoration therefore hangs off the pilot entity being <em>added</em> to a store, not
 * removed from one — see {@code DroneLifecycleSystems.RestoreParkedBodyOnAdd}. One mechanism
 * covers landing, switching worlds mid-flight, and crashing then logging back in.
 */
public final class ParkedBody implements Component<EntityStore> {

    /** Component id used for persistence. Must stay stable — it is a save-file key. */
    public static final String ID = "FpvParkedBody";

    public static final BuilderCodec<ParkedBody> CODEC =
            BuilderCodec.builder(ParkedBody.class, ParkedBody::new)
                    .append(
                            new KeyedCodec<>("AddedInvulnerable", Codec.BOOLEAN),
                            (parked, value, extraInfo) -> parked.addedInvulnerable = value,
                            (parked, extraInfo) -> parked.addedInvulnerable)
                    .add()
                    .append(
                            new KeyedCodec<>("AddedIntangible", Codec.BOOLEAN),
                            (parked, value, extraInfo) -> parked.addedIntangible = value,
                            (parked, extraInfo) -> parked.addedIntangible)
                    .add()
                    .build();

    private boolean addedInvulnerable;
    private boolean addedIntangible;

    /** For the codec. */
    private ParkedBody() {}

    public ParkedBody(boolean addedInvulnerable, boolean addedIntangible) {
        this.addedInvulnerable = addedInvulnerable;
        this.addedIntangible = addedIntangible;
    }

    /** Whether we added {@code Invulnerable} and must therefore take it away again. */
    public boolean addedInvulnerable() {
        return this.addedInvulnerable;
    }

    /** Whether we added {@code Intangible} and must therefore take it away again. */
    public boolean addedIntangible() {
        return this.addedIntangible;
    }

    @Nonnull
    @Override
    public Component<EntityStore> clone() {
        return new ParkedBody(this.addedInvulnerable, this.addedIntangible);
    }
}
