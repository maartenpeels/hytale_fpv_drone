pluginManagement {
    repositories {
        gradlePluginPortal()
        mavenCentral()
        maven {
            name = "AzureDoom Maven"
            url = uri("https://maven.azuredoom.com/mods")
        }
    }
}

plugins {
    id("org.gradle.toolchains.foojay-resolver-convention") version "1.0.0"
}

rootProject.name = "Hytale FPV Drone Mod"

// See CLAUDE.md, decision 10. The split is enforced by the build:
//   :fpv-core   - pure Java, zero Hytale dependencies, fully unit-tested.
//   :fpv-plugin - the Hytale plugin. Adapters only.
// :fpv-core cannot import com.hypixel.* because Hytale is not on its classpath.
include("fpv-core")
include("fpv-plugin")
