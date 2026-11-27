#include "mirrorcore.h"

#include <filesystem>
#include <fstream>
#include <iostream>

static void writeFile(const std::filesystem::path& path, const std::string& text) {
    std::ofstream out(path, std::ios::binary);
    if (!out) throw std::runtime_error("cannot write " + path.string());
    out << text;
}

int main(int argc, char** argv) {
    using namespace mirror;
    const std::filesystem::path outputDir = argc > 1 ? argv[1] : ".";
    std::filesystem::create_directories(outputDir);

    MirrorParameters mirror;
    const Mat3 rotation = kukaABCToMatrix({0.0, -20.8, 0.0});
    const Vec3 translation{1440.0, -70.0, 520.0};
    const double r = 250.0;
    const double z = asphereSag(mirror, r);
    auto toWorld = [&](const Vec3& p) { return rotation * p + translation; };

    FivePoints points{
        toWorld({0,0,0}),
        toWorld({r,0,z}),
        toWorld({-r,0,z}),
        toWorld({0,r,z}),
        toWorld({0,-r,z})
    };

    TrajectoryOptions trajectory;
    trajectory.pathPitchMm = 20.0;
    trajectory.pointSpacingMm = 10.0;
    trajectory.maxPoints = 20000;

    KrlOptions krl;
    krl.programName = "MIRROR01";
    const GeneratedOutput generated = generateKrlOutput(points, mirror, trajectory, krl);

    writeFile(outputDir / "MIRROR01.src", generated.srcCode);
    writeFile(outputDir / "MIRROR01.dat", generated.datCode);
    writeFile(outputDir / "MIRROR01_trajectory.csv", generated.csvText);
    writeFile(outputDir / "MIRROR01_report.txt", generated.reportText);
    std::cout << "Generated " << generated.points.size() << " points\n";
    return 0;
}
