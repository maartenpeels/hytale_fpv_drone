package com.maartenpeels.fpv.plugin.input;

import com.maartenpeels.fpv.control.PilotInputSample;

import javax.annotation.Nonnull;
import javax.annotation.Nullable;
import java.util.Set;
import java.util.UUID;
import java.util.concurrent.ConcurrentHashMap;
import java.util.concurrent.atomic.AtomicLong;

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
/*
 * Non-final so a test can substitute a failing offer. Nothing inside ClientMovementWatcher.accept
 * can be made to throw from the outside -- by design, every sanitising rule lives in the core
 * mapper, which never throws -- so the watcher's error counter would otherwise be the one part of
 * #49's instrument that is itself unverifiable. An instrument you cannot test is what #47 already
 * had.
 */
public class PilotInputBuffer {

    @Nonnull
    private final ConcurrentHashMap<UUID, PilotInputSlot> slots = new ConcurrentHashMap<>();

    /**
     * Offers that found a slot, and offers that did not. Netty writes, the world thread reads.
     *
     * <p>These exist because the drop is <em>correct behaviour that looks exactly like a bug</em>.
     * {@link #offer} writing existing slots only is what keeps this map bounded, but it means a
     * pilot whose key does not match produces no log line, no exception and no counter — #47 spent
     * four flights indistinguishable from "the client is not sending". Separating "nothing arrived"
     * from "everything arrived and was thrown away" is the whole point of #49, and it needs exactly
     * these two numbers.
     */
    @Nonnull
    private final AtomicLong offersAccepted = new AtomicLong();
    @Nonnull
    private final AtomicLong offersDropped = new AtomicLong();

    /**
     * Records a sample against a pilot, if that pilot is flying. <b>Netty thread.</b>
     *
     * <p>Silently drops input for anyone without an open slot, which is every player who is not
     * currently flying a drone. That is the normal case, not an error — but it is now counted, see
     * {@link #offersDropped()}.
     */
    public void offer(@Nonnull UUID pilotId, @Nonnull PilotInputSample sample) {
        PilotInputSlot slot = this.slots.get(pilotId);
        if (slot != null) {
            slot.offer(sample);
            this.offersAccepted.incrementAndGet();
        } else {
            this.offersDropped.incrementAndGet();
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

    /** Offers that landed in a slot, since startup. */
    public long offersAccepted() {
        return this.offersAccepted.get();
    }

    /** Offers discarded for want of a slot, since startup. */
    public long offersDropped() {
        return this.offersDropped.get();
    }

    /**
     * The pilots with an open slot, as a snapshot.
     *
     * <p>For printing beside the UUID the watcher is offering on: if a key mismatch ever does occur,
     * the two lists side by side end the question immediately, which reading alone could not do.
     */
    @Nonnull
    public Set<UUID> openKeys() {
        return Set.copyOf(this.slots.keySet());
    }
}
