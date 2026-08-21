// :fpv-plugin -- the Hytale plugin. Adapters only. See CLAUDE.md decision 10.
//
// Everything that touches com.hypixel.* lives here: ECS components and systems, packet
// handling, commands, UI pages, persistence, entity lifecycle. Flight and race logic belongs
// in :fpv-core. If you are writing a physics or race rule in this module, it is in the wrong
// place.

plugins {
// Uncomment if you are using IntelliJ.
//  idea
    java
    id("com.azuredoom.hytale-tools") version "1.+"
}

hytaleTools {
    javaVersion = property("java_version").toString().toInt()
    hytaleVersion = property("hytale_version").toString()
    manifestServerVersion = property("manifestServerVersion").toString()
    manifestGroup = property("manifest_group").toString()
    modId = property("mod_id").toString()
    modDescription = property("mod_description").toString()
    modUrl = property("mod_url").toString()
    mainClass = property("main_class").toString()
    modCredits = property("mod_author").toString()
    manifestDependencies = property("manifest_dependencies").toString()
    manifestOptionalDependencies = property("manifest_opt_dependencies").toString()
    curseforgeId = property("curseforgeID").toString()
    disabledByDefault = property("disabled_by_default").toString().toBoolean()
    includesPack = property("includes_pack").toString().toBoolean()
    patchline = property("patchline").toString()
    injectServerJavadocsIntoSources = property("injectServerJavadocsIntoSources").toString().toBoolean()
    generateAssetsBinary = property("generateAssetsBinary").toString().toBoolean()
}

// The Hytale server loads one jar, so :fpv-core's classes have to travel inside the plugin
// jar. `bundledCore` is resolved and unpacked into the jar below. It is non-transitive on
// purpose: :fpv-core has no runtime dependencies and must stay that way.
val bundledCore: Configuration by configurations.creating {
    isCanBeConsumed = false
    isCanBeResolved = true
    isTransitive = false
}

dependencies {
    implementation(project(":fpv-core"))
    bundledCore(project(":fpv-core"))

    // Hytale's ECS is drivable in a plain JVM -- no server boot, no Assets.zip, no client.
    // See docs/plans/34.md for what that does and does not make testable.
    //
    // The server jar has to be put on the test classpath by hand: the Hytale Gradle plugin
    // wires `vineServerJar` into `compileOnly` only (HytaleConfigurationConfigurer.groovy:66)
    // and has no test support of any kind. `files(...)` rather than
    // `testImplementation.extendsFrom(vineServerJar)` because a Test task needs the jar on the
    // RUNTIME classpath too, and `compileOnly` by definition is on no runtime classpath.
    // `vineServerJar` is declared `canBeResolved = true` (same file, :30), which is the hook.
    testImplementation(files(configurations.named("vineServerJar")))

    // Same junit-bom version as :fpv-core, so the two modules cannot drift.
    testImplementation(platform("org.junit:junit-bom:5.11.4"))
    testImplementation("org.junit.jupiter:junit-jupiter")
    testRuntimeOnly("org.junit.platform:junit-platform-launcher")
}

tasks.named<Test>("test") {
    // MANDATORY -- without this no ECS test can even load its own imports.
    //
    // ComponentRegistry holds a `static final HytaleLogger` (component/ComponentRegistry.java:76),
    // and HytaleLogger's static block (logger/HytaleLogger.java:112-126) throws
    //     IllegalStateException: Log manager wasn't set!
    // unless the JUL log manager was replaced at JVM startup. Calling System.setProperty from
    // inside a test is too late: java.util.logging.LogManager has already initialised.
    //
    // If you see that exception, you are running these tests on a JVM that did not get this
    // flag -- a hand-rolled Test task, or an IDE runner that ignores Gradle's jvmArgs.
    jvmArgs("-Djava.util.logging.manager=com.hypixel.hytale.logger.backend.HytaleLogManager")

    // Do NOT enable JUnit parallel execution here. `Store` binds to its constructing thread
    // (component/Store.java:106) and assertThread() (:1992-1997) throws a plain
    // IllegalStateException -- not a Java `assert`, so -da will not help -- from any other
    // thread. Parallel execution turns these tests into confusing intermittent failures.
}

tasks.named<Jar>("jar") {
    archiveBaseName.set(rootProject.property("mod_name").toString())
    archiveVersion.set(rootProject.property("version").toString())

    // `elements` is a Provider, so this carries the dependency on :fpv-core:jar rather than
    // resolving the configuration eagerly at configuration time.
    dependsOn(bundledCore)
    from(bundledCore.elements.map { artifacts -> artifacts.map { zipTree(it.asFile) } })
}

// Uncomment if you are using IntelliJ.
// idea {
//     module {
//         isDownloadSources = true
//         isDownloadJavadoc = true
//     }
// }
