#pragma once

#include <cstddef>
#include <string>
#include <vector>

namespace mirror {

constexpr double kPi = 3.1415926535897932384626433832795;
constexpr double kEps = 1e-12;

double degToRad(double degrees);
double radToDeg(double radians);

struct Vec3 {
    double x{0.0};
    double y{0.0};
    double z{0.0};

    Vec3 operator+(const Vec3& other) const;
    Vec3 operator-(const Vec3& other) const;
    Vec3 operator-() const;
    Vec3 operator*(double scalar) const;
    Vec3 operator/(double scalar) const;
    Vec3& operator+=(const Vec3& other);
};

Vec3 operator*(double scalar, const Vec3& value);
double dot(const Vec3& a, const Vec3& b);
Vec3 cross(const Vec3& a, const Vec3& b);
double norm(const Vec3& value);
Vec3 normalized(const Vec3& value);

struct Mat3 {
    double m[3][3]{};

    static Mat3 identity();
    static Mat3 fromColumns(const Vec3& c0, const Vec3& c1, const Vec3& c2);
};

Vec3 operator*(const Mat3& matrix, const Vec3& vector);
Mat3 operator*(const Mat3& a, const Mat3& b);
Mat3 transpose(const Mat3& matrix);
double determinant(const Mat3& matrix);

struct ABC {
    double a{0.0};
    double b{0.0};
    double c{0.0};
};

struct Frame {
    Vec3 p;
    ABC abc;
};

Mat3 kukaABCToMatrix(const ABC& abc);
ABC matrixToKukaABC(const Mat3& matrix);
ABC unwrapABC(const ABC& current, const ABC& previous);

struct FivePoints {
    Vec3 p0;
    Vec3 pxPlus;
    Vec3 pxMinus;
    Vec3 pyPlus;
    Vec3 pyMinus;
};

struct Diagnostics {
    double xPairRadiusMismatch{0.0};
    double yPairRadiusMismatch{0.0};
    double exDotEy{0.0};
    double exDotEz{0.0};
    double eyDotEz{0.0};
    double determinantR{0.0};
    double xMidCrossTrack{0.0};
    double yMidCrossTrack{0.0};
    double xMidSag{0.0};
    double yMidSag{0.0};
};

struct RealBase {
    Vec3 exW;
    Vec3 eyW;
    Vec3 ezW;
    Mat3 R_WM;
    Vec3 T_WM;
    Frame kukaBase;
};

RealBase buildRealBase(const FivePoints& points, Diagnostics& diagnostics);
Vec3 pointMToW(const RealBase& base, const Vec3& pointM);
Vec3 vectorMToW(const RealBase& base, const Vec3& vectorM);
Vec3 pointWToM(const RealBase& base, const Vec3& pointW);
Vec3 vectorWToM(const RealBase& base, const Vec3& vectorW);
Frame poseMToW(const RealBase& base, const Frame& poseM);
Frame poseWToM(const RealBase& base, const Frame& poseW);

enum class PathType {
    Spiral,
    ConcentricOutsideIn,
    Raster
};

enum class ToolZMode {
    IntoSurface,
    TowardOpening
};

enum class OrientationReference {
    FixedMirrorX,
    PathTangent
};

struct MirrorParameters {
    double diameterMm{660.0};
    double radiusOfCurvatureMm{349.0};
    double conicK{-0.60117};
};

struct TrajectoryOptions {
    PathType pathType{PathType::Spiral};
    double edgeMarginMm{5.0};
    double pathPitchMm{12.0};
    double pointSpacingMm{5.0};
    double normalOffsetMm{0.0};
    double approachClearanceMm{20.0};
    double rollOffsetDeg{0.0};
    bool reversePath{false};
    ToolZMode toolZMode{ToolZMode::IntoSurface};
    OrientationReference orientationReference{OrientationReference::FixedMirrorX};
    std::size_t maxPoints{20000};
};

struct KrlOptions {
    std::string programName{"MIRROR01"};
    int toolIndex{1};
    int baseIndex{5};
    double velocityCp{0.02};
    double accelerationCp{0.2};
    double approximationCdisMm{0.5};
    bool useCdis{true};
    bool writeBaseDataAtRuntime{false};
};

struct TrajectoryPoint {
    std::size_t index{0};
    Vec3 surfacePointM;
    Vec3 pointM;
    Vec3 openNormalM;
    Vec3 toolXAxisM;
    Vec3 toolYAxisM;
    Vec3 toolZAxisM;
    ABC abcM;
};

struct GeneratedOutput {
    RealBase base;
    Diagnostics diagnostics;
    std::vector<TrajectoryPoint> points;
    Frame approachFrameM;
    Frame retractFrameM;
    std::string srcCode;
    std::vector<std::string> warnings;
};

double asphereSag(const MirrorParameters& mirror, double radialDistanceMm);
Vec3 concaveOpeningNormal(const MirrorParameters& mirror, const Vec3& surfacePointM);
std::vector<Vec3> generateSurfacePath(const MirrorParameters& mirror,
                                      const TrajectoryOptions& options);
std::vector<TrajectoryPoint> buildOrientedTrajectory(const MirrorParameters& mirror,
                                                     const TrajectoryOptions& options);
GeneratedOutput generateKrlOutput(const FivePoints& fivePoints,
                                  const MirrorParameters& mirror,
                                  const TrajectoryOptions& trajectory,
                                  const KrlOptions& krl);

std::string formatFrame(const Frame& frame, int precision = 6);
std::string sanitizeKrlIdentifier(const std::string& value);

} // namespace mirror
