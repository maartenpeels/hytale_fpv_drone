// :fpv-core -- pure flight and race logic. See CLAUDE.md decision 10.
//
// HARD RULE: this module must never depend on Hytale. No `com.hypixel.*` import can compile
// here, because the Hytale Gradle plugin is not applied and the server jar is not on the
// classpath. That is the enforcement -- do not "fix" a compile error here by adding Hytale.
//
// Everything in here is a deterministic function of (state, input, dt) and is unit-tested
// without a running server:
//   quad physics integrator, PID controller, rate/expo curves, swept gate crossing,
//   race state machine, PilotProfile validation, leaderboard model.

plugins {
    `java-library`
}

dependencies {
    testImplementation(platform("org.junit:junit-bom:5.11.4"))
    testImplementation("org.junit.jupiter:junit-jupiter")
    testRuntimeOnly("org.junit.platform:junit-platform-launcher")
}
