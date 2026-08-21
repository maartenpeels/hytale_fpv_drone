# FPV Drone

A Hytale server plugin that adds simulated FPV (first-person view) quadcopter flight.

Not a creative-mode fly toy: the drone is a **real flight model simulated on the server** — thrust,
angular rates, a PID rate loop, drag and momentum — so it behaves like a quad and can be tuned like
one. Everything else the project is aiming at is built on that: rate/expo and PID tuning per pilot,
race courses with custom gates, authoritative lap timing, leaderboards, and spectating another
pilot's drone.

Java 25 · Hytale server `>=0.5.3 <0.6.0` · MIT

---

## Status: phase 0, not yet flyable

The project's first milestone exists to answer one question honestly, before anything is built on
top of it:

> Does a server-simulated drone at 30 TPS, with a network round trip on every input, feel flyable
> at all?

That question is still **open**. Where things stand:

- **`:fpv-core` has the flight model.** A rate-agnostic `QuadIntegrator`, the `RatePid` rate loop,
  Betaflight-style `RateCurve`/`RateProfile` stick curves, `MotorMixer`, and the `Quat`/`Vec3`
  math underneath — each with unit tests.
- **`:fpv-plugin` is barely started.** The entry point, config, and a `/fpv` command that prints
  the resolved tick/substep settings so you can confirm the plugin loaded. **There is no drone
  entity, no camera attach and no `/fpv launch` yet.**

The remaining phase-0 work is tracked under
[milestone v0.1 — Feel](https://github.com/maartenpeels/hytale_fpv_drone/milestone/1). Later
milestones (tuning, racing, multiplayer, long-range flight) are epics on the
[issue list](https://github.com/maartenpeels/hytale_fpv_drone/issues).

---

## Requirements

- **JDK 25.** Note this is needed to **run Gradle**, not just to compile — the
  `hytale-gradle-plugin` buildscript itself targets 25. If your default JDK is older, every Gradle
  command below needs a prefix:

  ```bash
  JAVA_HOME=$(/usr/libexec/java_home -v 25) ./gradlew build
  ```

  On macOS, `/usr/libexec/java_home -v 25` reporting no match means there is no JDK 25 installed
  and no prefix will help — install one first (`brew install --cask temurin@25`).
- **JetBrains Runtime is recommended** if you want hot reload while the dev server is running.
  `./gradlew hytaleJvmDoctor` reports which JVM Gradle resolved and whether enhanced class
  redefinition is available.
- A **Hytale installation**, for its `Assets.zip`. The standard per-OS install location is
  auto-detected; you only need to configure anything if yours is somewhere unusual.
- No Gradle install needed — the wrapper is committed.

## Build and test

```bash
./gradlew build              # full build, including tests
./gradlew :fpv-core:test     # the physics and curve tests on their own — fast, no server needed
```

`:fpv-core` has no Hytale dependency, so its tests run in about a second. That is where nearly all
the logic lives, and it is the loop to use while working on flight behaviour. Narrow it to one file
while iterating:

```bash
./gradlew :fpv-core:test --tests "*RateCurveTest"
```

The plugin jar lands at:

```
fpv-plugin/build/libs/FPV Drone-0.0.1.jar
```

**Note the path** — `fpv-plugin/build/libs/`, *not* the root `build/libs/`. The root `shadowJar`
task exists only as a compatibility shim for tooling that expects the old pre-module-split
location. Also note the space in the filename; that is
[#27](https://github.com/maartenpeels/hytale_fpv_drone/issues/27).

There is no lint or format command in this repo — no Spotless, Checkstyle or formatter plugin is
configured.

## Running a dev server

```bash
./gradlew setupHytaleDev                          # once, to sync the local Hytale dev environment
./gradlew runServer                               # run a local dev server with the plugin loaded
./gradlew runServer -Ddebug=true -Dhotswap=true   # ... with debugging and hot swap
```

`runServer` executes the Hytale server straight from the Gradle classpath with `--assets` pointed
at your install's `Assets.zip`, working directory `run/`. It does not build or read a packaged
server distribution, and it does not copy assets anywhere.

On Windows, use `gradlew.bat` in place of `./gradlew`.

## Configuration

The plugin writes `fpv_drone.json` into its data directory, holding just the two simulation-rate
knobs: `WorldTps` (default 30, Hytale's own) and `PhysicsSubsteps` (default 8, so ~4 ms per
integration step). Raising `WorldTps` is the documented escape hatch if flight turns out to feel
floaty — that it stays a config change rather than a rewrite is a deliberate constraint. Per-pilot
tuning is not server config; it will be a persisted profile.

`/fpv` in game prints the resolved values, which is the quickest check that your build actually
loaded.

Mod identity — id, version, entry point, dependencies, license — lives in `gradle.properties` and
is the single source for both modules. `manifest.json` is **generated** from it; never hand-edit
it:

```bash
./gradlew updatePluginManifest
```

## Layout

Two Gradle modules, and the split is load-bearing:

```
fpv-core/      pure Java + JUnit 5. Zero Hytale dependencies.
fpv-plugin/    the Hytale plugin. Adapters only. Depends on :fpv-core.
```

**`:fpv-core`** holds everything that can be reasoned about and tested without a game: the quad
integrator, PID controller, rate/expo curves, swept gate-crossing math, race state machine, pilot
profile validation, leaderboard model. It is deterministic — physics is a function of
`(state, input, dt)`.

**`:fpv-plugin`** holds only the boundary: ECS components and systems, packet handling, commands,
UI pages, persistence, entity lifecycle. It translates Hytale packets into core's types and core's
output back into packets.

The reason is blunt: the Hytale server API is undocumented, decompiled, pinned to a version range,
and still changing. When it breaks, only the adapter layer should break. The build enforces this —
the Hytale Gradle plugin is applied *only* in `:fpv-plugin`, so a `com.hypixel.*` import in
`:fpv-core` fails to compile with "package does not exist". At package time, `:fpv-core`'s classes
are unpacked into the plugin jar, because the server loads a single jar.

```
build.gradle.kts       shared Java/test config only — the root is not itself a Java project
settings.gradle.kts    module includes
gradle.properties      single source of mod identity
CLAUDE.md              the durable engineering context (see below)
docs/plans/            one design document per issue
```

## Contributing

Read **[CLAUDE.md](CLAUDE.md)** first. It is written for AI agents but it is the real engineering
documentation for this repo, and it will save you from the traps: verified facts read out of the
decompiled server (input channels, camera packets, world units — gravity is `32.0`, not `9.81`),
the settled design decisions and why the obvious alternatives were rejected, and the
always/ask-first/never boundaries on which files may be changed. Also read
[HOW-WE-WORK-WITH-AI.md](HOW-WE-WORK-WITH-AI.md) for how work moves from issue to merged PR.

The one workflow rule that matters most:

> **The Hytale API is undocumented. Do not guess at it — read it.** Decompiled server sources are
> on disk under `~/.gradle/caches/hytale-decompiled/`. Grep them and confirm the class, method and
> signature exist before writing anything that touches `com.hypixel.*`.

Practically:

- Work from a GitHub issue; each non-trivial change gets a design note in `docs/plans/<issue>.md`.
- Branch as `<type>/<slug>` (`feat/`, `fix/`, `chore/`, `docs/`); never push to `main`.
- Every physics, curve, crossing and race-state change lands with a unit test. Flying around is not
  verification — these are cheap to test and impossible to eyeball.
- CI runs `./gradlew build` on JDK 25 for every push and pull request to `main`.

## License

MIT — `mod_license` in `gradle.properties`. See `LICENSE`.
