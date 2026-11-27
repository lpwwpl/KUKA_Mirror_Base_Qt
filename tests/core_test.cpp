#include "mirrorcore.h"

#include <cmath>
#include <iostream>

int main() {
    using namespace mirror;
    const ABC abc{0.0, -20.8, 0.0};
    const Mat3 R = kukaABCToMatrix(abc);
    const Vec3 T{1440.0, -70.0, 520.0};
    const double r = 250.0;
    const MirrorParameters mp{};
    const double sag = asphereSag(mp, r);
    auto toW = [&](const Vec3& p) { return R * p + T; };
    FivePoints f{
        toW({0,0,0}), toW({r,0,sag}), toW({-r,0,sag}),
        toW({0,r,sag}), toW({0,-r,sag})
    };
    Diagnostics d;
    RealBase base = buildRealBase(f,d);
    std::cout << formatFrame(base.kukaBase) << "\n";
    if (std::abs(base.kukaBase.abc.b + 20.8) > 1e-6) return 2;

    TrajectoryOptions tr;
    tr.edgeMarginMm = 300.0;
    tr.pathPitchMm = 10.0;
    tr.pointSpacingMm = 5.0;
    tr.maxPoints = 1000;
    KrlOptions ko;
    GeneratedOutput out = generateKrlOutput(f, mp, tr, ko);
    std::cout << "points=" << out.points.size() << "\n";
    std::cout << out.srcCode.substr(0, 240) << "\n";
    return out.points.empty() ? 3 : 0;
}
