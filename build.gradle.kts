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
