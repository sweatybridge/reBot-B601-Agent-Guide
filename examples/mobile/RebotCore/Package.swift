// swift-tools-version:5.9
import PackageDescription

let package = Package(
    name: "RebotCore",
    platforms: [.iOS(.v16), .macOS(.v13)],
    products: [
        .library(name: "RebotCore", targets: ["RebotCore"]),
    ],
    targets: [
        .target(name: "RebotCore"),
        .testTarget(
            name: "RebotCoreTests",
            dependencies: ["RebotCore"],
            resources: [.copy("Fixtures")]
        ),
    ]
)
