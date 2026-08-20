// Root project. Deliberately not a Java project and deliberately not a Hytale plugin --
// it only holds configuration shared by :fpv-core and :fpv-plugin.
//
// The Hytale Gradle plugin is applied ONLY in :fpv-plugin, which is what keeps Hytale off
// :fpv-core's compile classpath. See CLAUDE.md decision 10.

plugins {
// Uncomment if you are using IntelliJ.
//  idea
}

val javaVersion = property("java_version").toString().toInt()

// Compatibility shim for external tooling that invokes `shadowJar` and/or reads the plugin
// jar from the ROOT `build/libs`. Neither is a Gradle default here: this project has never
// had a `shadowJar` task, and the module split moved the jar to `fpv-plugin/build/libs`.
//
// A separate shadow/fat-jar step is not needed -- `:fpv-plugin:jar` is already fat. It
// bundles the Hytale asset-editor runtime (added by hytale-tools) and unpacks :fpv-core via
// the `bundledCore` configuration. So this only aliases that task and republishes its output
// at the pre-split location.
//
// Delete this if the external tooling is pointed at `:fpv-plugin:jar` instead.
tasks.register<Copy>("shadowJar") {
    group = "build"
    description = "Alias for :fpv-plugin:jar, copied to the root build/libs for external tooling."
    from(project(":fpv-plugin").tasks.named<Jar>("jar"))
    into(layout.buildDirectory.dir("libs"))
}

subprojects {
    group = rootProject.property("group").toString()
    version = rootProject.property("version").toString()

    pluginManager.withPlugin("java") {
        extensions.configure<JavaPluginExtension> {
            toolchain.languageVersion.set(JavaLanguageVersion.of(javaVersion))
        }

        repositories {
            mavenCentral()
        }

        tasks.withType<Test>().configureEach {
            useJUnitPlatform()
            testLogging {
                events("failed")
                exceptionFormat = org.gradle.api.tasks.testing.logging.TestExceptionFormat.FULL
            }
        }

        tasks.withType<Javadoc>().configureEach {
            (options as org.gradle.external.javadoc.StandardJavadocDocletOptions)
                .addStringOption("Xdoclint:-missing", "-quiet")
        }
    }
}
