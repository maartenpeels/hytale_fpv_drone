package com.maartenpeels.fpv.plugin.input;

import com.maartenpeels.fpv.control.ControlInput;
import com.maartenpeels.fpv.control.LookTrack;
import com.maartenpeels.fpv.control.PilotInputMapper;
import com.maartenpeels.fpv.control.PilotInputSample;
import com.maartenpeels.fpv.control.PilotInputUpdate;

import javax.annotation.Nonnull;
import javax.annotation.Nullable;
import java.util.concurrent.atomic.AtomicReference;

/**
 * One flying pilot's input, handed from the netty thread to the world thread.
 *
 * <h2>The thread split, which is the whole point of this class</h2>
 *
 * {@code ClientMovement} arrives on the netty thread, via
 * {@code PacketAdapters.registerInbound} — fired at
 * {@code server/core/io/netty/PlayerChannelHandler.java:30} for every inbound packet before dispatch.
 * The simulation runs on the world thread. A {@code Store} binds to its constructing thread and
 * {@code assertThread()} throws a plain {@code IllegalStateException} — not a Java {@code assert}, so
 * {@code -da} does not help — from any other, so **the netty side must not touch the ECS**.
 *
 * <p>So exactly one field crosses the boundary: {@link #pending}, written by
 * {@link #offer(PilotInputSample)} on the netty thread and taken by {@link #nextInput} on the world
 * thread. Everything else here is plain, non-volatile, and touched only by the world thread. Which
 * field is which is not decoration — get it wrong and the symptom is a rare torn read that looks like
 * a physics glitch.
 *
 * <p>The alternative was {@code world.execute(...)} per packet. Rejected because it makes the handoff
 * per <em>packet</em>: a 144 FPS client on a 30 TPS world would queue five runnables a tick that all
 * mutate the same state and only the last of which matters. A slot expresses "latest input wins" as a
 * data structure rather than as a queue that happens to be drained in order.
 *
 * <h2>The interval handed to the mapper</h2>
 *
 * {@link PilotInputMapper#map} is called <b>once per world tick</b> with {@code dt} = the tick length,
 * and the resulting {@link ControlInput} is held constant across every integration substep of that
 * tick. Three consequences, and each one is a rule below rather than an accident:
 *
 * <ol>
 *   <li><b>Not once per substep.</b> Only the tick's first substep would see a fresh sample; the rest
 *       would read no look and answer zero deflection, putting a spike through the rate PID at the
 *       substep frequency. Input is a per-tick quantity because the packet rate is not the substep
 *       rate.
 *   <li><b>A quiet tick ages the track; it does not re-consume the held sample.</b> Re-feeding the
 *       last sample looks equivalent — its look angles now equal the tracked ones, so the delta is
 *       zero — but {@code map} answers a {@link LookTrack} sampled <em>now</em>, resetting
 *       {@link LookTrack#secondsSinceSample}. Five quiet ticks then a real sample would divide a
 *       five-tick delta by one tick's {@code dt}: the exact over-report #17's self-review caught,
 *       reintroduced through the retry path. {@link PilotInputSample#withoutLook()} takes
 *       {@code map}'s ageing branch instead, so the interval accumulates and the wish axes still hold.
 *   <li><b>Several packets inside one tick collapse to the newest.</b> Its absolute look angles are
 *       differenced against the last <em>consumed</em> ones over one tick, so a high-frame-rate client
 *       gets finer resolution and no extra authority.
 * </ol>
 *
 * <h2>The staleness cutoff</h2>
 *
 * Holding the wish axes indefinitely would mean a frozen client — TCP buffered, process suspended,
 * not disconnected, so #18's teardown never fires — leaves the drone at whatever throttle it last had,
 * climbing away unattended. After {@link #MAX_HELD_SECONDS} without a fresh packet the slot switches
 * to {@link PilotInputMapper#centred}, which rests the throttle at <em>hover</em> rather than cutting
 * the motors — so the drone stops where it is instead of continuing to climb, which is what #45 fixed:
 * this cutoff used to hand back a mid-scale throttle, i.e. the very unattended climb it exists to
 * stop. In seconds rather than ticks, so it does not change meaning at 240 TPS.
 */
public final class PilotInputSlot {

    /**
     * How long the last packet's throttle and yaw keep being obeyed once packets stop.
     *
     * <p>A guess, and the first thing to adjust if #24 finds a hitching client getting its throttle
     * cut. Nothing depends on the exact figure; it only has to be long enough to ride out a stall and
     * short enough that an abandoned drone stops climbing.
     */
    public static final double MAX_HELD_SECONDS = 0.5;

    /** Netty writes, world thread takes. The only field that crosses threads. */
    private final AtomicReference<PilotInputSample> pending = new AtomicReference<>();

    /** World thread only. The last sample actually consumed, replayed while packets are missing. */
    @Nonnull
    private PilotInputSample held = PilotInputSample.EMPTY;

    /** World thread only. The look memory a delta is measured against. */
    @Nonnull
    private LookTrack track = LookTrack.UNSET;

    /** World thread only. Seconds of ticking since the last fresh packet. */
    private double heldSeconds;

    /**
     * Records the newest sample from this pilot. <b>Netty thread.</b>
     *
     * <p>Latest wins: an unread sample is overwritten rather than queued, because a stick position
     * superseded before anyone looked at it is not information. Allocates nothing and blocks on
     * nothing, which matters because a watcher's exceptions are swallowed and logged at SEVERE
     * ({@code PacketAdapters.java:104-106}) — anything that can fail here fails invisibly, so nothing
     * that can fail lives here.
     */
    public void offer(@Nonnull PilotInputSample sample) {
        this.pending.set(sample);
    }

    /**
     * The stick positions to fly this tick, and the look memory advanced. <b>World thread.</b>
     *
     * @param mapper the shared, stateless mapper; the per-pilot memory is this slot's
     * @param tickSeconds the tick's length in seconds — the interval this slot's look delta is
     *     measured over, accumulated across quiet ticks
     */
    @Nonnull
    public ControlInput nextInput(@Nonnull PilotInputMapper mapper, double tickSeconds) {
        PilotInputSample fresh = this.pending.getAndSet(null);

        PilotInputUpdate update;
        if (fresh != null) {
            this.held = fresh;
            this.heldSeconds = 0.0;
            update = mapper.map(this.track, fresh, tickSeconds);
        } else {
            this.heldSeconds += tickSeconds;
            update =
                    this.heldSeconds > MAX_HELD_SECONDS
                            ? mapper.centred(this.track, tickSeconds)
                            : mapper.map(this.track, this.held.withoutLook(), tickSeconds);
        }

        this.track = update.track();
        return update.input();
    }

    /** The look memory, for tests and for anything that wants to see the interval accumulate. */
    @Nonnull
    public LookTrack track() {
        return this.track;
    }

    /** Whether a packet is waiting to be consumed. Test seam; not a scheduling signal. */
    @Nullable
    PilotInputSample peekPending() {
        return this.pending.get();
    }
}
