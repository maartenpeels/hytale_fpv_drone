package com.maartenpeels.fpv.plugin.input;

import com.maartenpeels.fpv.control.PilotInputSample;

import javax.annotation.Nonnull;
import javax.annotation.Nullable;
import java.util.UUID;
import java.util.concurrent.ConcurrentHashMap;

/**
 * The set of pilots whose input we are collecting, and the slot each one's packets land in.
 *
 * <p>The netty thread offers samples; the world thread opens and closes slots and drains them. See
 * {@link PilotInputSlot} for why the handoff is a slot rather than a task queue.
 *
 * <h2>Presence of a key means "this pilot is flying"</h2>
 *
 * {@link #offer} writes an <b>existing</b> slot and does nothing otherwise. That single rule is what
 * keeps this map bounded: it holds exactly the pilots with a live drone, because only the world thread
 * — which knows — ever adds a key. A {@code computeIfAbsent} here would instead grow one entry for
 * every player who has ever sent a movement packet, and packet 108 arrives many times a second from
 * every connected player whether they care about drones or not.
 *
 * <h2>Why the key is a {@code UUID}</h2>
 *
 * The watcher has a {@code PlayerRef} and reads {@code getUuid()} — a plain final field, so safe from
 * the netty thread. The world thread has the pilot's entity and reads {@code UUIDComponent}. Verified
 * those are the same value: {@code Player.saveConfig} uses {@code UUIDComponent.getUuid()} as the
 * player-storage key (`server/core/entity/entities/Player.java:303-307`) and
 * {@code Player.isHiddenFromLivingEntity} compares it against the UUIDs {@code PlayerRef} manages
 * (`:648-655`), both under {@code assert uuidComponent != null}.
 *
 * <p>A {@code Ref<EntityStore>} would have been the obvious key and is wrong three times over: it does
 * not override {@code equals}, it is invalidated on every world switch, and touching one from the
 * netty thread is exactly the ECS access the split exists to prevent.
 */
public final class PilotInputBuffer {

    @Nonnull
    private final ConcurrentHashMap<UUID, PilotInputSlot> slots = new ConcurrentHashMap<>();

    /**
     * Records a sample against a pilot, if that pilot is flying. <b>Netty thread.</b>
     *
     * <p>Silently drops input for anyone without an open slot, which is every player who is not
     * currently flying a drone. That is the normal case, not an error.
     */
    public void offer(@Nonnull UUID pilotId, @Nonnull PilotInputSample sample) {
        PilotInputSlot slot = this.slots.get(pilotId);
        if (slot != null) {
            slot.offer(sample);
        }
    }

    /**
     * Starts collecting input for a pilot, discarding anything a previous session left behind.
     * <b>World thread.</b>
     *
     * <p>A fresh slot per launch is deliberate: a {@code LookTrack} carried over from a landing an hour
     * ago would make the first tick of the new flight a full-deflection flick from an angle nobody
     * chose.
     */
    @Nonnull
    public PilotInputSlot open(@Nonnull UUID pilotId) {
        PilotInputSlot slot = new PilotInputSlot();
        this.slots.put(pilotId, slot);
        return slot;
    }

    /** Stops collecting, and forgets the look memory. <b>World thread.</b> Idempotent. */
    public void close(@Nonnull UUID pilotId) {
        this.slots.remove(pilotId);
    }

    /** The pilot's slot, or {@code null} if they are not flying. <b>World thread.</b> */
    @Nullable
    public PilotInputSlot slotOf(@Nonnull UUID pilotId) {
        return this.slots.get(pilotId);
    }

    /** How many pilots are being collected for. Exists so a test can assert the map does not grow. */
    public int size() {
        return this.slots.size();
    }
}
