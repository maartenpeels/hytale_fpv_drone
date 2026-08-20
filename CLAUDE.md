# FPV Drone — Hytale Server Plugin

A Hytale server plugin that adds simulated FPV (first-person view) quadcopter flight, with
real flight physics, tunable rates/PID, race courses with custom gates, leaderboards, and
spectating.

This file is the durable context for working in this repo. It records **decisions that are
settled** (do not relitigate them without saying so explicitly) and **facts verified against
the decompiled server** (trust these over guesses).

---

## Commands

```bash
./gradlew setupHytaleDev          # sync local Hytale dev environment (run once)
./gradlew runServer               # run local dev server
./gradlew runServer -Ddebug=true -Dhotswap=true
./gradlew hytaleJvmDoctor         # verify JVM / hotswap setup
./gradlew build                   # build the plugin
./gradlew :fpv-core:test          # unit tests — fast, no server needed
./gradlew updatePluginManifest    # regenerate manifest.json from gradle.properties
./gradlew build --refresh-dependencies
```

Java 25 — required to **run Gradle**, not just to compile, because the `hytale-gradle-plugin`
buildscript targets 25. If the default JDK is older, prefix with
`JAVA_HOME=$(/usr/libexec/java_home -v 25)`. JetBrains Runtime recommended for hot reload.

---

## The single most important workflow rule

**The Hytale API is undocumented. Do not guess at it — read it.**

Decompiled server sources are on disk:

```
~/.gradle/caches/hytale-decompiled/release-0.5.9/server/com/hypixel/hytale/
```

Before writing any code that touches `com.hypixel.*`, grep that tree and confirm the class,
method, field and signature actually exist. Every "Verified facts" entry below was read out
of those sources; anything not listed there has not been checked.

Useful landmarks:

| Area | Path under `com/hypixel/hytale/` |
| --- | --- |
| Plugin/API entry | `server/core/plugin/JavaPlugin.java` |
| Protocol packets | `protocol/packets/**` |
| ECS store & systems | `component/**`, `server/core/modules/**` |
| World, TPS, chunks | `server/core/universe/world/**` |
| Chunk streaming | `server/core/modules/entity/player/ChunkTracker.java` |
| Collision | `server/core/modules/collision/**`, `math/raycast/**` |
| Server-driven UI | `server/core/ui/**` + any `builtin/**/*Page.java` |
| Player persistence | `server/core/universe/playerdata/**` |
| Asset types | `server/core/asset/type/**` |

---

## Verified facts about the Hytale server (release-0.5.9)

These are read from the decompiled sources, not inferred. They constrain the design.

### Input

- `ClientMovement` (packet 108, client→server) carries: `movementStates`,
  `relativePosition`, `absolutePosition`, `bodyOrientation`, `lookOrientation`,
  `teleportAck`, **`wishMovement` (a `Position`, i.e. an analog desired-movement vector)**,
  `velocity`, `mountedTo`, `riderMovementStates`.
- `wishMovement` + `lookOrientation` are the only continuous input channels available.
- `MovementStates` is **23 booleans** (`idle`, `jumping`, `flying`, `sprinting`, `crouching`,
  `gliding`, …). There are no analog aux channels — **no AUX switches, no fifth axis.**
- `Direction` is `{float yaw, float pitch, float roll}` everywhere in the protocol. Roll is
  representable.
- **`Direction`'s signs disagree with `ControlInput`'s, on two axes out of three.** Read from
  `Vector3dUtil.setYawPitch` (`math/vector/Vector3dUtil.java:57-63`), which builds a direction
  vector as `x = -sin(yaw)·cos(pitch)`, `y = sin(pitch)`, `z = -cos(yaw)·cos(pitch)`. Therefore:
  - `y = sin(pitch)` means **Hytale's `pitch` is positive nose-*up***, the opposite of
    `ControlInput.pitch`, which is transmitter convention and positive nose-*down*. Any
    camera or entity-rotation adapter **must negate it**.
  - at yaw 0 the direction is `(0, 0, -1)`, so **forward is `−Z`** — which is why `:fpv-core`
    picked `−Z` forward too, keeping this boundary a sign flip rather than a 180° offset.
  - yaw increases toward `−X`, i.e. **opposite to a right-handed yaw about `+Y`**. Also negate.
- **Controllers are not supported by Hytale yet.** Expected in update 5. Until then keyboard
  and mouse drive `wishMovement`/`lookOrientation`, which quantises the stick vector to
  corners. Packet handling should not need to change when controllers land.

### Camera

- `SetServerCamera` (packet 280, server→client): `clientCameraView`, `isLocked`,
  `cameraSettings`.
- `ServerCameraSettings` includes `attachedToType` + `attachedToEntityId`, `isFirstPerson`,
  `rotationType`/`rotation` (a `Direction`, so **roll works**), `positionType`/`position`,
  `positionOffset`/`rotationOffset`, `positionLerpSpeed`/`rotationLerpSpeed`,
  `skipCharacterPhysics`, `sendMouseMotion`, `allowPitchControls`, `displayReticle`,
  `canMoveType`, `movementForceRotationType`.
- This is the mechanism for both FPV view and spectating.

### Movement / physics

- `UpdateMovementSettings` (packet 110) pushes a **65-field** `MovementSettings` per player:
  `mass`, `dragCoefficient`, `acceleration`, `airDragMin/Max` + speed curves,
  `airControlMin/MaxMultiplier`, `canFly`, `invertedGravity`,
  `horizontalFlySpeed`/`verticalFlySpeed`, `wishDirectionGravityX/Y`,
  `wishDirectionWeightX/Y`, and many per-gait multipliers.
- It is a **character controller model**: no angular rates, no roll authority, no thrust
  vector. It cannot express a quadcopter. We do not build flight on it.
- `MountMovement` (packet 166) has the **client** sending the mount's `absolutePosition` —
  mounts are client-authoritative, so "drone as mount" would hand physics to the client where
  we have no code. Not usable.
- Collision is available server-side: `server/core/modules/collision/` has
  `MovingBoxBoxCollisionEvaluator` (**swept AABB**), `CollisionMath`, `BlockCollisionData`,
  `CollisionResult`, `CollisionFilter`; plus `math/raycast/RaycastAABB`.

#### World units and axes

These three set the units `:fpv-core` works in. Getting them wrong is not a crash, it is a
drone that flies wrong in a way nobody can point at.

- **Gravity is `32.0`, not `9.81`.** `PhysicsConstants.GRAVITY_ACCELERATION = 32.0`
  (`server/core/modules/physics/util/PhysicsConstants.java:4`). Hytale's world is **not
  metric-consistent** — if a block is a metre, gravity is ~3.3× real. A drone modelled on real
  gravity feels lunar next to everything else in the world. `QuadParameters.DEFAULT_GRAVITY`
  matches the server figure deliberately; do not "fix" it to 9.81.
- **`+Y` is up.** `ForceProviderStandard` computes `-gravity * mass` and adds it to `force.y`
  (`server/core/modules/physics/util/ForceProviderStandard.java:36,45`).
- **`protocol.Position` is `double x, y, z`** (`protocol/Position.java:17-19`) — *not* float,
  despite `Direction` being float. So `:fpv-core` carrying position and velocity as `double`
  costs nothing at the packet boundary, and avoids the drift `float` accumulates at ~240
  integration steps per second.

### Tick rate

- `World.setTps(int)` is per-world, settable **up to 2048**, and pushes a `SetUpdateRate`
  packet so the client is told the new rate.
- **Default is 30 TPS** (`WorldTpsResetCommand`), i.e. 33 ms per tick. Commands: `/world tps`
  (alias `tickrate`), `/world tps reset`.

### Chunk loading

- Server-side residency: `WorldChunk.addKeepLoaded()` / `removeKeepLoaded()` is a refcounted
  ticket (`AtomicInteger`). `WorldConfig.ChunkConfig.keepLoadedRegion` is a `Box2D`.
  `ChunkUnloadingSystem` honours both.
- Client-side streaming: `ChunkTracker` is a per-player `Component<EntityStore>`.
  `PlayerChunkTrackerSystems.UpdateSystem` queries
  `Query.and(ChunkTracker, Player, PlayerRef, TransformComponent)` and drives streaming off
  the **player's own `TransformComponent`**.
- **There is no API to redirect chunk streaming to another position.** The only supported
  lever is moving the player entity. (See decision 5.)
- Bandwidth ceilings: `MAX_CHUNKS_PER_SECOND = 36` (remote), `128` (LAN), `256` (local);
  `MAX_CHUNKS_PER_TICK = 4`; `MAX_HOT_LOADED_CHUNKS_RADIUS = 8`; `MIN_LOADED_CHUNKS_RADIUS = 2`.
  Adjustable at runtime via `ChunkMaxSendRateCommand`.
- **Consequence:** at view radius 8, crossing one chunk boundary needs ~17 new columns, so a
  remote pilot sustains roughly 2 boundary crossings/sec. Fast pilots will outrun terrain
  streaming. Expect to tune send rate and view radius.

### Assets

Asset types available to the pack (`server/core/asset/type/`) include: `model`, `modelvfx`,
`trail`, `particle`, `blocktype`, `blockset`, `camera`, `gamemode`, `physicalmaterial`,
`projectile`, `portalworld`, and **`responsecurve`** — the last is the natural substrate for
expo/rate curves.

`includes_pack = true` is already set in `gradle.properties`.

### UI

Server-driven, document + selector based. `server/core/ui/builder/UICommandBuilder` exposes
`append(documentPath)`, `appendInline(selector, document)`, `insertBefore*`, `clear`,
`remove`, and `set(selector, value)` overloads for `String`/`Message`/`boolean`/`int`/`float`/
`double`/arrays/lists. `UIEventBuilder` handles events back. Follow the many
`builtin/**/*Page.java` examples (`ShopPage`, `BiomeEditorPage`, `PrefabSavePage`).

### Persistence

- Plugin config: `this.withConfig("name", CODEC)` returning `Config<T>`, with `BuilderCodec`
  / `KeyedCodec` from `com.hypixel.hytale.codec`.
- Per-player: `server/core/universe/playerdata/` — `PlayerStorage`, `PlayerStorageProvider`,
  `DiskPlayerStorageProvider`.

### Trigger volumes (evaluated, not used)

`builtin/triggervolumes/` provides `BoxShape`/`SphereShape`/`CylinderShape`, groups,
conditions, cooldowns, enter/exit effects, an in-game authoring tool, and `SetKeepLoaded`.
**Rejected for gates** because displays are editor wireframes (no custom texturing), detection
is tick-granularity containment (tunnels at speed), and it expresses neither crossing
direction nor gate ordering.

---

## Settled decisions

Do not silently revisit these. If new evidence contradicts one, say so explicitly and
re-decide.

1. **Input** — bet on native gamepad passthrough when Hytale update 5 lands. Develop and test
   with keyboard/mouse now. Read `ClientMovement.wishMovement` + `lookOrientation`; normalise
   at the plugin boundary into `ControlInput` (`throttle` 0..1 because a quad's throttle is
   unidirectional; `roll`/`pitch`/`yaw` −1..1) so `:fpv-core` never sees a packet.
2. **Physics runs on the server.** A real quad model — thrust, angular rates, PID, drag,
   momentum — simulated server-side. This is what makes rates/PID (goal 3), calibration
   (goal 2), and authoritative race timing (goals 6–7) real rather than cosmetic. It costs a
   full network round trip on every input; that is accepted.
3. **Sim rate: substep inside the 30 TPS tick.** Run the integrator at N fixed substeps per
   tick (e.g. 8 × ~4 ms). The integrator must be **rate-agnostic — `step(dt)`** — with substep
   count *and* world TPS as config values.
   *Known limitation:* substepping fixes sim fidelity but **not** control latency or visual
   smoothness, both of which stay pinned at 30 Hz. If flight feels floaty, the escape hatch is
   `World.setTps(120..240)` on a dedicated drone world — a config change, not a rewrite.
   Keeping that cheap is a standing constraint.
4. **Embodiment: separate drone entity, body parked.** The server spawns a drone entity with a
   pack model. `SetServerCamera` attaches the pilot's camera to it with full yaw/pitch/roll and
   `skipCharacterPhysics`. The pilot's character is invisible and invulnerable. Chosen because
   `ClientMovement.absolutePosition` is client-authoritative, so driving the player entity
   directly means a per-tick authority fight and rubber-banding.
5. **Chunks: the body follows the drone silently.** Because there is no API to redirect
   `ChunkTracker` (verified above), the pilot's body is teleported to the drone each tick
   *purely* to drag the client's chunk window along. Jitter on the body is never rendered
   because the camera is attached to the drone. Layer `addKeepLoaded()` tickets on top for
   server-side residency. Custom chunk streaming was considered and rejected: it means
   reimplementing a core subsystem against internals with no contract.
6. **Gates are custom entities with pack models, detected by our own math.** Rendering is a
   `model` asset per gate type. Detection is a **swept segment/AABB test** from last-tick to
   this-tick position against the gate plane, with a signed normal for direction and a
   per-race expected-gate index for ordering. Tunnel-proof by construction. Reuse the same
   swept routine as terrain collision.
7. **Tuning goes through custom UI documents**, authored in the asset pack and driven by
   selector. Non-negotiable companion rule: tune state is a **pure `PilotProfile` model with
   its own validation** in `:fpv-core`; UI is a front-end over it. PID logic must never live
   in a selector callback.
8. **Course data is plugin-owned and authoritative.** A `Course` (world id, ordered gates with
   position/orientation/type/size, start gate, finish gate, lap count) is persisted as codec
   JSON. Gate entities are **disposable views** spawned from that data on world load. Never
   derive course identity or order from world entity state — rollbacks and chunk unloads would
   silently corrupt courses and orphan leaderboards.
9. **Multiplayer target: ~4–8 concurrent pilots. Terrain collision only.** Drones collide with
   blocks (swept AABB → crash → respawn) and pass through each other. Removes the O(n²) pass
   and, more importantly, removes who-hit-whom fairness disputes from the leaderboards.
10. **Two Gradle modules, enforced by the build.**
    - `:fpv-core` — **zero Hytale dependencies.** Quad physics integrator, PID controller,
      rate/expo curves, swept gate crossing, race state machine, `PilotProfile` validation,
      leaderboard model. Fully unit-tested, runs in milliseconds.
    - `:fpv-plugin` — depends on `:fpv-core` + Hytale. Adapters only: ECS components and
      systems, packet handling, commands, UI pages, persistence, entity lifecycle.

    Rationale: the API is decompiled, undocumented, pinned to `>=0.5.3 <0.6.0`, and changing
    at update 5. When it breaks, only the adapter layer should break.

    **If you are about to add a `com.hypixel.*` import to `:fpv-core`, the design is wrong.**
11. **Calibration wizard (goal 2) is blocked upstream.** With keyboard input there are no
    stick endpoints, centre, deadband, or channel order to calibrate. Design for it; do not
    build it until controllers exist.

---

## Roadmap

Ordered so the project-killing unknown is attacked first: *does a server-simulated drone at
30 TPS, with a round trip on every input, feel flyable at all?* Nothing downstream matters if
the answer is no.

**Phase 0 — Feel. This is v0.1.**
`:fpv-core` with a tested integrator, PID and rate curves. `:fpv-plugin` with `/fpv launch`,
`/fpv land`, drone entity, camera attach with roll, body-follow for chunks, terrain collision
and crash. **Nothing else** — no gates, no races, no UI, no leaderboards, no persistence
beyond a default tune. Ship it, fly it, and answer the question honestly. If it fails, reopen
decision 3 before building anything on top.

Later phases, roughly in dependency order:

- Rates/PID tuning + `PilotProfile` persistence + tuning UI (goal 3)
- Gate entities, gate types, swept crossing (goals 6, 8) — circle, square, pillar,
  horizontal hole; note a **pillar** gate is passed *around*, which is a side test, not a
  crossing test
- Course authoring, race state machine, timing, leaderboards (goals 6, 7)
- Multiple concurrent courses and race sessions in one world (goal 7)
- Multiplayer sessions and pack flying (goal 5)
- Spectating — camera attach to another pilot's drone (goal 9); largely falls out of decision 4
- Long-range flight tuning: keepLoaded ticket cursor, chunk send rate, view radius (goal 4)
- Controller support and calibration wizard (goals 1, 2) — **gated on Hytale update 5**

---

## Conventions

- **Java**, matching the template. Package root `com.maartenpeels.fpv`.
- Plugin entry point is `com.maartenpeels.FPVDrone` (per `main_class` in `gradle.properties`).
- `:fpv-core` is pure: no Hytale imports, no I/O, no statics holding mutable state. Physics and
  race logic are deterministic functions of `(state, input, dt)`.
- Every physics, curve, crossing and race-state change lands with a unit test. These are cheap
  to test and impossible to verify by flying around.
- Prefer values and explicit state machines over booleans-plus-branching for race and flight
  mode state.
- Config goes through `BuilderCodec`/`KeyedCodec` — see `FpvConfig` for the pattern.

---

## Project layout

```
build.gradle.kts        root: shared Java/test config only. Not a Java project, not a plugin.
settings.gradle.kts     includes both modules
gradle.properties       single source of mod identity; both modules read from it

fpv-core/               pure Java + JUnit 5. NO Hytale on the classpath.
  src/main/java/com/maartenpeels/fpv/**
  src/test/java/com/maartenpeels/fpv/**

fpv-plugin/             applies com.azuredoom.hytale-tools; depends on :fpv-core
  src/main/java/com/maartenpeels/FPVDrone.java        entry point (matches main_class)
  src/main/java/com/maartenpeels/fpv/plugin/**        adapters: config, commands, systems
  src/main/resources/manifest.json                    generated; do not hand-edit
```

Notes that matter when changing the build:

- The Hytale Gradle plugin is applied **only** in `:fpv-plugin`. That is the entire
  enforcement mechanism for decision 10 — a `com.hypixel.*` import in `:fpv-core` fails to
  compile with "package does not exist". Verified, not assumed.
- The server loads a single jar, so `:fpv-core`'s classes are unpacked into the plugin jar via
  the `bundledCore` configuration in `fpv-plugin/build.gradle.kts`. It is deliberately
  non-transitive: `:fpv-core` must stay dependency-free at runtime. **If you add a runtime
  dependency to `:fpv-core`, the plugin jar will silently ship without it.**
- Unqualified task names resolve across the build, so `./gradlew runServer`,
  `./gradlew updatePluginManifest` and `./gradlew setupHytaleDev` still work from the root
  without a `:fpv-plugin:` prefix.
- **The plugin jar moved.** It is built at `fpv-plugin/build/libs/`, not `build/libs/` as it
  was before the split. The root `shadowJar` task exists purely as a compatibility shim for
  external tooling that expects the old task name and the old path — it aliases
  `:fpv-plugin:jar` and copies the result to the root `build/libs/`. There is no Shadow plugin
  and no separate fat-jar step; `:fpv-plugin:jar` is already fat.
- The `hytale-gradle-plugin` buildscript requires **JVM 25 to run Gradle itself**, not just to
  compile. On a machine whose default JDK is older:
  `JAVA_HOME=$(/usr/libexec/java_home -v 25) ./gradlew build`.
- **The local-install override resolves `Assets.zip` only.** Every usage in the Gradle plugin
  is assets-related; it has no effect on the server jar. Two traps, both verified in
  `hytale-gradle-plugin-1.0.48` sources:
  - The gradle property is **`hytale_home`** (or `hytools.hytale.home`) —
    `HytaleExtensionDefaults` conventions `ext.hytaleHomeOverride` from *those* names. A
    property literally named `hytaleHomeOverride` is read by nothing, so setting it in
    `~/.gradle/gradle.properties` is a silent no-op. `hytaleHomeOverride` is the *extension
    field*, assignable only from a build script.
  - You almost certainly do not need it. When the override is empty,
    `DownloadAssetsZipTask.resolveLocalAssetsZip` auto-detects per OS —
    `~/Library/Application Support/Hytale` on macOS, `~/AppData/Roaming/Hytale` on Windows.
    Only a non-standard install path needs the override. What setting it changes is that
    `runServer` reads the install's `Assets.zip` directly and stops depending on
    `downloadAssetsZip`.

### How `runServer` actually works

Worth knowing before hand-rolling any launcher. `runServer` is a `JavaExec` that runs
`com.hypixel.hytale.Main` with a classpath of your compiled classes + resources + runtime deps
+ `vineServerJar` (the Maven artifact `com.hypixel.hytale:Server`), passing
`--assets=<absolute path>`, with working directory `run/`. `prepareRunServer` stages mod jars
into `run/mods`.

**It never creates or reads a `HytaleServer.jar`, and never copies `Assets.zip`.** A packaged
server distribution does exist — `~/Library/Application Support/Hytale/install/release/package/game/latest/Server/`
holds `HytaleServer.jar` and `HytaleServer.aot.config` — but the Gradle dev flow does not use
it. Do not construct that layout by hand to satisfy external tooling; point the tooling at
`:fpv-plugin:jar` and `runServer` instead.

---

## Known repo issues (unfixed)

- The plugin jar is named from `mod_name`, so it builds as `FPV Drone-0.0.1.jar` — with a
  space. Switch `archiveBaseName` to `mod_id` if that ever causes trouble.
- No `LICENSE` file, though `mod_license = MIT`.
- `README.md` is still the unmodified Hytale Plugin Template readme.

---

<!-- BEGIN ai-native-blueprint v0.1.1 (installed 2026-08-20). Team-owned; edit freely. -->

## Feedback loops

Every command below was executed at install time on this repo and passed.

```bash
./gradlew :fpv-core:test --tests "*ControlInputTest"   # file-scoped: ~1s
./gradlew :fpv-core:test                               # all core tests
./gradlew build                                        # full build, ~5s warm
```

Prefix with `JAVA_HOME=$(/usr/libexec/java_home -v 25)` if the default JDK is older. If
`/usr/libexec/java_home -v 25` reports no match, there is no JDK 25 on the machine and no
prefix will help — install one first (`brew install --cask temurin@25`, or JetBrains Runtime
for hot reload; this box has JBR 25.0.4 and OpenJDK 25.0.1). Not run at install time.

No lint or format command exists in this repo — no Spotless, Checkstyle or formatter plugin is
configured. Do not invent one; add a real plugin first if you want a lint loop.

`./gradlew setupHytaleDev` and `./gradlew runServer` are **not** verified here: the first
mutates the local Hytale dev environment, the second blocks on a running server.

## Tests

- Runner: JUnit 5, `:fpv-core` only (`:fpv-plugin` has no tests — adapters are verified by flying)
- Naming: `<ClassUnderTest>Test.java`, mirroring the main package
- Exemplary test file — copy its conventions (`@Nested` per behaviour group, test names that
  state the *reason*, e.g. `rejectsNegativeThrottleBecauseThrottleIsUnidirectional`):
  `fpv-core/src/test/java/com/maartenpeels/fpv/control/ControlInputTest.java`

## Boundaries

**Always** (proceed without asking):

- `fpv-core/src/**` — with a unit test in the same change
- `fpv-plugin/src/main/java/**` — adapters
- `docs/plans/**`, `docs/adr/**`

**Ask first:**

- `gradle.properties` — single source of mod identity; changing it moves the jar name, package
  and manifest
- `*.gradle.kts` and `settings.gradle.kts` — the build *is* the enforcement mechanism for
  decision 10
- `.github/workflows/**`
- The "Settled decisions" section above — decisions are re-decided out loud, per its own rule
- `.claude/settings.json`, `.claude/skills/**`

**Never:**

- Add a `com.hypixel.*` import to `:fpv-core` (decision 10 — if you want to, the design is wrong)
- Hand-edit `fpv-plugin/src/main/resources/manifest.json` — regenerate with
  `./gradlew updatePluginManifest`
- Edit anything under `~/.gradle/caches/hytale-decompiled/**` — read-only reference
- Write to `build/`, `run/`, `server/`, or commit anything from them
- Push to `main` directly
- Write a command into this file that you have not executed

## Team tools

- Tracker: GitHub Issues — `gh issue view <n>`, `gh issue create --title <t> --body <b>`,
  subtasks are separate issues referencing the parent (GitHub has no native subtask)
- VCS: GitHub — `gh pr create --base <branch> --fill`, `gh pr view <n>`, `gh pr diff <n>`
- Auth: `gh` keyring, account **`maartenpeels`** (scopes `repo`, `read:org`, `gist`); git over
  SSH. The keyring also holds `maartenpeels2`, which is **not a collaborator** on this repo —
  active under it, `gh pr create` fails with "must be a collaborator". Check with
  `gh auth status`; switch with `gh auth switch -u maartenpeels`.
- No Jira in this project — `jira` is on PATH for other work; ignore it here

## Git workflow

- Branches: `<type>/<slug>` (`feat/`, `fix/`, `chore/`); never push to `main` directly
- Worktrees: one per task in `.worktrees/<task-id>` (gitignored), one agent per worktree, run
  `./gradlew build` at create time to confirm the environment, clean up after merge. The
  already-checked-out branch counts as the current task's worktree — create new worktrees only
  for additional parallel tasks. See the `parallel-work` skill.
- Agent workflow and skills: see `HOW-WE-WORK-WITH-AI.md`

<!-- END ai-native-blueprint -->

