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
    // hytaleHomeOverride = property("hytaleHomeOverride").toString()
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
