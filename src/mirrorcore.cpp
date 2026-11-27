#include "mirrorcore.h"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <iomanip>
#include <limits>
#include <locale>
#include <sstream>
#include <stdexcept>

namespace mirror {

namespace {

double clampValue(double value, double minimum, double maximum) {
    return std::max(minimum, std::min(value, maximum));
}

double unwrapAngle(double value, double reference) {
    while (value - reference > 180.0) value -= 360.0;
    while (value - reference < -180.0) value += 360.0;
    return value;
}

std::string number(double value, int precision = 6) {
    if (std::abs(value) < 0.5 * std::pow(10.0, -precision)) value = 0.0;
    std::ostringstream stream;
    stream.imbue(std::locale::classic());
    stream << std::fixed << std::setprecision(precision) << value;
    return stream.str();
}

Mat3 rotateFrameAboutLocalZ(const Mat3& frame, double rollDegrees) {
    const double angle = degToRad(rollDegrees);
    const double c = std::cos(angle);
    const double s = std::sin(angle);

    const Vec3 x{frame.m[0][0], frame.m[1][0], frame.m[2][0]};
    const Vec3 y{frame.m[0][1], frame.m[1][1], frame.m[2][1]};
    const Vec3 z{frame.m[0][2], frame.m[1][2], frame.m[2][2]};

    const Vec3 xr = c * x + s * y;
    const Vec3 yr = -s * x + c * y;
    return Mat3::fromColumns(xr, yr, z);
}

Vec3 fallbackTangent(const Vec3& toolZ) {
    Vec3 reference{1.0, 0.0, 0.0};
    if (std::abs(dot(reference, toolZ)) > 0.9) reference = {0.0, 1.0, 0.0};
    return normalized(reference - dot(reference, toolZ) * toolZ);
}

} // namespace

double degToRad(double degrees) { return degrees * kPi / 180.0; }
double radToDeg(double radians) { return radians * 180.0 / kPi; }

Vec3 Vec3::operator+(const Vec3& other) const { return {x + other.x, y + other.y, z + other.z}; }
Vec3 Vec3::operator-(const Vec3& other) const { return {x - other.x, y - other.y, z - other.z}; }
Vec3 Vec3::operator-() const { return {-x, -y, -z}; }
Vec3 Vec3::operator*(double scalar) const { return {x * scalar, y * scalar, z * scalar}; }
Vec3 Vec3::operator/(double scalar) const {
    if (std::abs(scalar) < kEps) throw std::runtime_error("除数接近零");
    return {x / scalar, y / scalar, z / scalar};
}
Vec3& Vec3::operator+=(const Vec3& other) {
    x += other.x;
    y += other.y;
    z += other.z;
    return *this;
}

Vec3 operator*(double scalar, const Vec3& value) { return value * scalar; }
double dot(const Vec3& a, const Vec3& b) { return a.x*b.x + a.y*b.y + a.z*b.z; }
Vec3 cross(const Vec3& a, const Vec3& b) {
    return {
        a.y*b.z - a.z*b.y,
        a.z*b.x - a.x*b.z,
        a.x*b.y - a.y*b.x
    };
}
double norm(const Vec3& value) { return std::sqrt(dot(value, value)); }
Vec3 normalized(const Vec3& value) {
    const double length = norm(value);
    if (length < kEps) throw std::runtime_error("无法归一化零向量");
    return value / length;
}

Mat3 Mat3::identity() {
    Mat3 matrix;
    matrix.m[0][0] = 1.0;
    matrix.m[1][1] = 1.0;
    matrix.m[2][2] = 1.0;
    return matrix;
}

Mat3 Mat3::fromColumns(const Vec3& c0, const Vec3& c1, const Vec3& c2) {
    Mat3 matrix;
    matrix.m[0][0] = c0.x; matrix.m[1][0] = c0.y; matrix.m[2][0] = c0.z;
    matrix.m[0][1] = c1.x; matrix.m[1][1] = c1.y; matrix.m[2][1] = c1.z;
    matrix.m[0][2] = c2.x; matrix.m[1][2] = c2.y; matrix.m[2][2] = c2.z;
    return matrix;
}

Vec3 operator*(const Mat3& matrix, const Vec3& vector) {
    return {
        matrix.m[0][0]*vector.x + matrix.m[0][1]*vector.y + matrix.m[0][2]*vector.z,
        matrix.m[1][0]*vector.x + matrix.m[1][1]*vector.y + matrix.m[1][2]*vector.z,
        matrix.m[2][0]*vector.x + matrix.m[2][1]*vector.y + matrix.m[2][2]*vector.z
    };
}

Mat3 operator*(const Mat3& a, const Mat3& b) {
    Mat3 result;
    for (int row = 0; row < 3; ++row) {
        for (int column = 0; column < 3; ++column) {
            for (int k = 0; k < 3; ++k) {
                result.m[row][column] += a.m[row][k] * b.m[k][column];
            }
        }
    }
    return result;
}

Mat3 transpose(const Mat3& matrix) {
    Mat3 result;
    for (int row = 0; row < 3; ++row)
        for (int column = 0; column < 3; ++column)
            result.m[row][column] = matrix.m[column][row];
    return result;
}

double determinant(const Mat3& matrix) {
    return
        matrix.m[0][0] * (matrix.m[1][1]*matrix.m[2][2] - matrix.m[1][2]*matrix.m[2][1]) -
        matrix.m[0][1] * (matrix.m[1][0]*matrix.m[2][2] - matrix.m[1][2]*matrix.m[2][0]) +
        matrix.m[0][2] * (matrix.m[1][0]*matrix.m[2][1] - matrix.m[1][1]*matrix.m[2][0]);
}

Mat3 kukaABCToMatrix(const ABC& abc) {
    const double a = degToRad(abc.a);
    const double b = degToRad(abc.b);
    const double c = degToRad(abc.c);
    const double ca = std::cos(a), sa = std::sin(a);
    const double cb = std::cos(b), sb = std::sin(b);
    const double cc = std::cos(c), sc = std::sin(c);

    Mat3 matrix;
    matrix.m[0][0] = ca*cb;
    matrix.m[0][1] = ca*sb*sc - sa*cc;
    matrix.m[0][2] = ca*sb*cc + sa*sc;
    matrix.m[1][0] = sa*cb;
    matrix.m[1][1] = sa*sb*sc + ca*cc;
    matrix.m[1][2] = sa*sb*cc - ca*sc;
    matrix.m[2][0] = -sb;
    matrix.m[2][1] = cb*sc;
    matrix.m[2][2] = cb*cc;
    return matrix;
}

ABC matrixToKukaABC(const Mat3& matrix) {
    const double b = std::atan2(-matrix.m[2][0],
                                std::hypot(matrix.m[0][0], matrix.m[1][0]));
    const double cb = std::cos(b);
    double a = 0.0;
    double c = 0.0;

    if (std::abs(cb) > 1e-10) {
        a = std::atan2(matrix.m[1][0], matrix.m[0][0]);
        c = std::atan2(matrix.m[2][1], matrix.m[2][2]);
    } else {
        c = 0.0;
        if (b > 0.0) a = std::atan2(matrix.m[1][2], matrix.m[0][2]);
        else a = std::atan2(-matrix.m[1][2], -matrix.m[0][2]);
    }
    return {radToDeg(a), radToDeg(b), radToDeg(c)};
}

ABC unwrapABC(const ABC& current, const ABC& previous) {
    return {
        unwrapAngle(current.a, previous.a),
        current.b,
        unwrapAngle(current.c, previous.c)
    };
}

RealBase buildRealBase(const FivePoints& points, Diagnostics& diagnostics) {
    Vec3 ex = normalized(points.pxPlus - points.pxMinus);
    const Vec3 vyRaw = points.pyPlus - points.pyMinus;
    const Vec3 vyOrthogonal = vyRaw - dot(vyRaw, ex) * ex;
    Vec3 ey = normalized(vyOrthogonal);
    Vec3 ez = normalized(cross(ex, ey));

    const Vec3 midX = 0.5 * (points.pxPlus + points.pxMinus);
    const Vec3 midY = 0.5 * (points.pyPlus + points.pyMinus);
    const Vec3 cavityHint = 0.5 * (midX + midY) - points.p0;
    if (norm(cavityHint) < 1e-9) {
        throw std::runtime_error("无法判断凹面开口方向：对称点中点与 P0 太接近");
    }
    if (dot(ez, cavityHint) < 0.0) {
        ey = -ey;
        ez = -ez;
    }

    const Mat3 rotation = Mat3::fromColumns(ex, ey, ez);
    const Vec3 translation = points.p0;

    const double rxp = norm(points.pxPlus - points.p0);
    const double rxm = norm(points.pxMinus - points.p0);
    const double ryp = norm(points.pyPlus - points.p0);
    const double rym = norm(points.pyMinus - points.p0);
    diagnostics.xPairRadiusMismatch = std::abs(rxp - rxm);
    diagnostics.yPairRadiusMismatch = std::abs(ryp - rym);
    diagnostics.exDotEy = std::abs(dot(ex, ey));
    diagnostics.exDotEz = std::abs(dot(ex, ez));
    diagnostics.eyDotEz = std::abs(dot(ey, ez));
    diagnostics.determinantR = determinant(rotation);

    const Vec3 midXFromP0 = midX - points.p0;
    const Vec3 midYFromP0 = midY - points.p0;
    diagnostics.xMidCrossTrack = std::hypot(dot(midXFromP0, ex), dot(midXFromP0, ey));
    diagnostics.yMidCrossTrack = std::hypot(dot(midYFromP0, ex), dot(midYFromP0, ey));
    diagnostics.xMidSag = dot(midXFromP0, ez);
    diagnostics.yMidSag = dot(midYFromP0, ez);

    return {ex, ey, ez, rotation, translation,
            {translation, matrixToKukaABC(rotation)}};
}

Vec3 pointMToW(const RealBase& base, const Vec3& pointM) {
    return base.R_WM * pointM + base.T_WM;
}
Vec3 vectorMToW(const RealBase& base, const Vec3& vectorM) {
    return base.R_WM * vectorM;
}
Vec3 pointWToM(const RealBase& base, const Vec3& pointW) {
    return transpose(base.R_WM) * (pointW - base.T_WM);
}
Vec3 vectorWToM(const RealBase& base, const Vec3& vectorW) {
    return transpose(base.R_WM) * vectorW;
}
Frame poseMToW(const RealBase& base, const Frame& poseM) {
    const Mat3 rotationWT = base.R_WM * kukaABCToMatrix(poseM.abc);
    return {pointMToW(base, poseM.p), matrixToKukaABC(rotationWT)};
}
Frame poseWToM(const RealBase& base, const Frame& poseW) {
    const Mat3 rotationMT = transpose(base.R_WM) * kukaABCToMatrix(poseW.abc);
    return {pointWToM(base, poseW.p), matrixToKukaABC(rotationMT)};
}

double asphereSag(const MirrorParameters& mirror, double radialDistanceMm) {
    if (mirror.radiusOfCurvatureMm <= 0.0) {
        throw std::runtime_error("曲率半径 R 必须大于 0");
    }
    if (radialDistanceMm < 0.0) {
        throw std::runtime_error("径向距离不能为负数");
    }
    const double r2 = radialDistanceMm * radialDistanceMm;
    const double R2 = mirror.radiusOfCurvatureMm * mirror.radiusOfCurvatureMm;
    const double radicand = 1.0 - (1.0 + mirror.conicK) * r2 / R2;
    if (radicand < -1e-10) {
        throw std::runtime_error("非球面参数在当前口径处无实数矢高，请检查 D、R、k");
    }
    const double root = std::sqrt(std::max(0.0, radicand));
    const double denominator = mirror.radiusOfCurvatureMm * (1.0 + root);
    if (std::abs(denominator) < kEps) {
        throw std::runtime_error("非球面矢高公式分母接近零");
    }
    return r2 / denominator;
}

Vec3 concaveOpeningNormal(const MirrorParameters& mirror, const Vec3& surfacePointM) {
    const Vec3 unnormalized{
        -surfacePointM.x,
        -surfacePointM.y,
        mirror.radiusOfCurvatureMm - (1.0 + mirror.conicK) * surfacePointM.z
    };
    return normalized(unnormalized);
}

std::vector<Vec3> generateSurfacePath(const MirrorParameters& mirror,
                                      const TrajectoryOptions& options) {
    if (mirror.diameterMm <= 0.0) throw std::runtime_error("镜面直径必须大于 0");
    if (options.edgeMarginMm < 0.0) throw std::runtime_error("边缘余量不能为负数");
    if (options.pathPitchMm <= 0.0) throw std::runtime_error("轨迹间距必须大于 0");
    if (options.pointSpacingMm <= 0.0) throw std::runtime_error("点间距必须大于 0");

    const double radiusMax = mirror.diameterMm * 0.5 - options.edgeMarginMm;
    if (radiusMax <= 0.0) throw std::runtime_error("边缘余量过大，可加工半径不大于 0");

    std::vector<Vec3> points;
    points.reserve(std::min<std::size_t>(options.maxPoints, 10000));

    auto appendPoint = [&](double x, double y) {
        if (points.size() >= options.maxPoints) {
            throw std::runtime_error("轨迹点数超过上限，请增大点间距/轨迹间距或提高最大点数");
        }
        const double r = std::hypot(x, y);
        points.push_back({x, y, asphereSag(mirror, r)});
    };

    if (options.pathType == PathType::Spiral) {
        const double radialPerRadian = options.pathPitchMm / (2.0 * kPi);
        double theta = 0.0;
        while (true) {
            const double radius = radialPerRadian * theta;
            if (radius > radiusMax) break;
            appendPoint(radius * std::cos(theta), radius * std::sin(theta));
            const double dsDtheta = std::sqrt(radius*radius + radialPerRadian*radialPerRadian);
            const double dtheta = clampValue(options.pointSpacingMm / std::max(dsDtheta, 1e-9),
                                             0.002, 0.25);
            theta += dtheta;
        }
        const double finalTheta = radiusMax / radialPerRadian;
        const Vec3 finalPoint{radiusMax * std::cos(finalTheta),
                              radiusMax * std::sin(finalTheta),
                              asphereSag(mirror, radiusMax)};
        if (points.empty() || norm(finalPoint - points.back()) > 0.1 * options.pointSpacingMm) {
            if (points.size() >= options.maxPoints) {
                throw std::runtime_error("轨迹点数超过上限");
            }
            points.push_back(finalPoint);
        }
    } else if (options.pathType == PathType::ConcentricOutsideIn) {
        // Continuous concentric-circle path from the outer edge to the center.
        // Viewed from the concave opening side (+Z_M), every ring runs
        // counter-clockwise. Each ring ends at the same polar angle, then a
        // short radial segment connects to the next inner ring.
        std::vector<double> radii;
        for (double radius = radiusMax; radius > 1e-9;
             radius -= options.pathPitchMm) {
            radii.push_back(radius);
        }
        if (radii.empty() || radii.back() > 1e-9) radii.push_back(0.0);

        const double direction = 1.0; // CCW when viewed from +Z_M (opening side)
        const double startAngle = 0.0;
        for (std::size_t ring = 0; ring < radii.size(); ++ring) {
            const double radius = radii[ring];
            if (radius < 1e-9) {
                if (points.empty() || std::hypot(points.back().x, points.back().y) > 1e-9) {
                    appendPoint(0.0, 0.0);
                }
                break;
            }

            const double circumference = 2.0 * kPi * radius;
            const int segments = std::max(24, static_cast<int>(std::ceil(circumference / options.pointSpacingMm)));
            const int firstSegment = (ring == 0 ? 0 : 1); // connector already wrote ring start
            for (int i = firstSegment; i <= segments; ++i) {
                const double fraction = static_cast<double>(i) / static_cast<double>(segments);
                const double angle = startAngle + direction * 2.0 * kPi * fraction;
                appendPoint(radius * std::cos(angle), radius * std::sin(angle));
            }

            if (ring + 1 < radii.size()) {
                const double nextRadius = radii[ring + 1];
                const int radialSegments = std::max(1, static_cast<int>(
                    std::ceil(std::abs(radius - nextRadius) / options.pointSpacingMm)));
                for (int i = 1; i <= radialSegments; ++i) {
                    const double fraction = static_cast<double>(i) / static_cast<double>(radialSegments);
                    const double connectorRadius = radius + (nextRadius - radius) * fraction;
                    appendPoint(connectorRadius * std::cos(startAngle),
                                connectorRadius * std::sin(startAngle));
                }
            }
        }
    } else {
        std::vector<double> rows;
        for (double y = -radiusMax; y < radiusMax; y += options.pathPitchMm) rows.push_back(y);
        rows.push_back(radiusMax);

        bool leftToRight = true;
        for (double y : rows) {
            const double xExtent = std::sqrt(std::max(0.0, radiusMax*radiusMax - y*y));
            const int segments = std::max(1, static_cast<int>(std::ceil(2.0*xExtent / options.pointSpacingMm)));
            for (int i = 0; i <= segments; ++i) {
                const double fraction = static_cast<double>(i) / static_cast<double>(segments);
                const double xForward = -xExtent + 2.0*xExtent*fraction;
                const double x = leftToRight ? xForward : -xForward;
                appendPoint(x, y);
            }
            leftToRight = !leftToRight;
        }
    }

    if (points.size() < 2) throw std::runtime_error("生成的轨迹点不足 2 个");
    if (options.reversePath) std::reverse(points.begin(), points.end());
    return points;
}

std::vector<TrajectoryPoint> buildOrientedTrajectory(const MirrorParameters& mirror,
                                                     const TrajectoryOptions& options) {
    const std::vector<Vec3> surfacePoints = generateSurfacePath(mirror, options);
    std::vector<Vec3> normals;
    std::vector<Vec3> processPoints;
    normals.reserve(surfacePoints.size());
    processPoints.reserve(surfacePoints.size());

    for (const Vec3& surface : surfacePoints) {
        const Vec3 normal = concaveOpeningNormal(mirror, surface);
        normals.push_back(normal);
        processPoints.push_back(surface + options.normalOffsetMm * normal);
    }

    std::vector<TrajectoryPoint> result;
    result.reserve(surfacePoints.size());
    Vec3 previousX{};
    bool hasPrevious = false;

    for (std::size_t i = 0; i < surfacePoints.size(); ++i) {
        Vec3 tangent;
        if (i == 0) tangent = processPoints[1] - processPoints[0];
        else if (i + 1 == surfacePoints.size()) tangent = processPoints[i] - processPoints[i - 1];
        else tangent = processPoints[i + 1] - processPoints[i - 1];

        const Vec3 toolZ = options.toolZMode == ToolZMode::IntoSurface ? -normals[i] : normals[i];
        Vec3 referenceDirection;
        if (options.orientationReference == OrientationReference::PathTangent) {
            referenceDirection = tangent;
        } else {
            referenceDirection = {1.0, 0.0, 0.0};
        }
        Vec3 tangentProjected = referenceDirection - dot(referenceDirection, toolZ) * toolZ;
        Vec3 toolX;
        if (norm(tangentProjected) < 1e-9) toolX = fallbackTangent(toolZ);
        else toolX = normalized(tangentProjected);
        Vec3 toolY = normalized(cross(toolZ, toolX));
        toolX = normalized(cross(toolY, toolZ));

        if (hasPrevious && dot(toolX, previousX) < 0.0) {
            toolX = -toolX;
            toolY = -toolY;
        }

        Mat3 toolRotation = Mat3::fromColumns(toolX, toolY, toolZ);
        toolRotation = rotateFrameAboutLocalZ(toolRotation, options.rollOffsetDeg);
        toolX = {toolRotation.m[0][0], toolRotation.m[1][0], toolRotation.m[2][0]};
        toolY = {toolRotation.m[0][1], toolRotation.m[1][1], toolRotation.m[2][1]};
        ABC abc = matrixToKukaABC(toolRotation);

        result.push_back({i + 1, surfacePoints[i], processPoints[i], normals[i],
                          toolX, toolY, toolZ, abc});
        previousX = toolX;
        hasPrevious = true;
    }
    return result;
}

std::string formatFrame(const Frame& frame, int precision) {
    return "{X " + number(frame.p.x, precision) +
           ",Y " + number(frame.p.y, precision) +
           ",Z " + number(frame.p.z, precision) +
           ",A " + number(frame.abc.a, precision) +
           ",B " + number(frame.abc.b, precision) +
           ",C " + number(frame.abc.c, precision) + "}";
}

std::string sanitizeKrlIdentifier(const std::string& value) {
    std::string output;
    output.reserve(value.size());
    for (char ch : value) {
        const unsigned char u = static_cast<unsigned char>(ch);
        if (std::isalnum(u) || ch == '_') output.push_back(static_cast<char>(std::toupper(u)));
    }
    if (output.empty() || !std::isalpha(static_cast<unsigned char>(output.front()))) {
        output.insert(output.begin(), 'P');
    }
    if (output.size() > 20) output.resize(20);
    return output;
}

GeneratedOutput generateKrlOutput(const FivePoints& fivePoints,
                                  const MirrorParameters& mirror,
                                  const TrajectoryOptions& trajectory,
                                  const KrlOptions& krlInput) {
    GeneratedOutput output;
    output.base = buildRealBase(fivePoints, output.diagnostics);
    output.points = buildOrientedTrajectory(mirror, trajectory);

    if (krlInput.toolIndex < 1 || krlInput.baseIndex < 1) {
        throw std::runtime_error("TOOL_DATA 和 BASE_DATA 编号必须大于等于 1");
    }

    const TrajectoryPoint& first = output.points.front();
    const TrajectoryPoint& last = output.points.back();
    output.approachFrameM = {
        first.pointM + trajectory.approachClearanceMm * first.openNormalM,
        first.abcM
    };
    output.retractFrameM = {
        last.pointM + trajectory.approachClearanceMm * last.openNormalM,
        last.abcM
    };

    if (output.diagnostics.xPairRadiusMismatch > 0.5)
        output.warnings.push_back("PX+ 与 PX- 到 P0 的距离差超过 0.5 mm");
    if (output.diagnostics.yPairRadiusMismatch > 0.5)
        output.warnings.push_back("PY+ 与 PY- 到 P0 的距离差超过 0.5 mm");
    if (output.diagnostics.xMidCrossTrack > 0.5 || output.diagnostics.yMidCrossTrack > 0.5)
        output.warnings.push_back("对称点中点存在较大横向偏差，请检查取点是否真正等半径对称");
    if (std::abs(output.diagnostics.xMidSag - output.diagnostics.yMidSag) > 1.0)
        output.warnings.push_back("X/Y 对称点矢高差超过 1 mm，请检查测量半径是否一致");
    if (output.points.size() > 10000)
        output.warnings.push_back("轨迹点超过 10000 个，KRL 文件较大，建议增大点间距或分区加工");
    if (std::abs(output.base.kukaBase.abc.b) > 85.0)
        output.warnings.push_back("BASE 的 B 角接近欧拉角奇异区，请核对坐标轴定义");

    double pathLength = 0.0;
    double maxNormalTilt = 0.0;
    double maxOrientationStep = 0.0;
    for (std::size_t i = 0; i < output.points.size(); ++i) {
        maxNormalTilt = std::max(maxNormalTilt,
            radToDeg(std::acos(clampValue(dot(output.points[i].openNormalM, Vec3{0.0, 0.0, 1.0}), -1.0, 1.0))));
        if (i > 0) {
            pathLength += norm(output.points[i].pointM - output.points[i - 1].pointM);
            const TrajectoryPoint& current = output.points[i];
            const TrajectoryPoint& previous = output.points[i - 1];
            const double traceRelative = dot(current.toolXAxisM, previous.toolXAxisM)
                                       + dot(current.toolYAxisM, previous.toolYAxisM)
                                       + dot(current.toolZAxisM, previous.toolZAxisM);
            const double cosine = clampValue((traceRelative - 1.0) * 0.5, -1.0, 1.0);
            maxOrientationStep = std::max(maxOrientationStep, radToDeg(std::acos(cosine)));
        }
    }
    if (maxOrientationStep > 10.0)
        output.warnings.push_back("相邻轨迹点实际姿态变化超过 10°，请减小点间距或检查姿态参考方式");

    KrlOptions krl = krlInput;
    krl.programName = sanitizeKrlIdentifier(krl.programName);

    std::ostringstream src;
    src.imbue(std::locale::classic());
    src << "&ACCESS RVP\n&REL 1\n&PARAM EDITMASK = *\nDEF " << krl.programName << "()\n\n";
    src << "  BAS(#INITMOV,0)\n";
    src << "  $TOOL = TOOL_DATA[" << krl.toolIndex << "]\n";
    src << "  $BASE = BASE_DATA[" << krl.baseIndex << "]\n";
    src << "  ; Calculated mirror BASE_DATA[" << krl.baseIndex << "] = "
        << formatFrame(output.base.kukaBase) << "\n";
    if (krl.writeBaseDataAtRuntime) {
        src << "  BASE_DATA[" << krl.baseIndex << "] = "
            << formatFrame(output.base.kukaBase) << "\n";
        src << "  $BASE = BASE_DATA[" << krl.baseIndex << "]\n";
    }
    src << "  $VEL.CP = " << number(krl.velocityCp, 4) << "\n";
    src << "  $ACC.CP = " << number(krl.accelerationCp, 4) << "\n";
    src << "  $APO.CDIS = " << number(krl.approximationCdisMm, 3) << "\n";
    src << "  $ADVANCE = 3\n\n";
    src << "  ; All path points below are local XYZABC values in the mirror BASE.\n";
    src << "  ; Do not add the mirror installation angle to B again.\n";
    src << "  ; SAFETY: verify the safe start pose, S/T branch, singularities and collisions.\n";
    src << "  ; Perform a dry run in T1 at low override before machining.\n\n";

    // SRC-only output: coordinates are written directly into LIN commands.
    // No external DAT file is required.
    src << "  LIN " << formatFrame(output.approachFrameM) << "\n";
    src << "  LIN " << formatFrame({output.points.front().pointM,
                                      output.points.front().abcM}) << "\n";
    for (std::size_t i = 1; i + 1 < output.points.size(); ++i) {
        src << "  LIN " << formatFrame({output.points[i].pointM,
                                          output.points[i].abcM});
        if (krl.useCdis) src << " C_DIS";
        src << "\n";
    }
    if (output.points.size() > 1) {
        src << "  LIN " << formatFrame({output.points.back().pointM,
                                          output.points.back().abcM}) << "\n";
    }
    src << "  LIN " << formatFrame(output.retractFrameM) << "\n\nEND\n";
    output.srcCode = src.str();

    return output;
}

} // namespace mirror
