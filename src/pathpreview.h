#pragma once

#include "mirrorcore.h"

#include <QWidget>
#include <vector>

class QTimerEvent;

class PathPreviewWidget : public QWidget {
public:
    explicit PathPreviewWidget(QWidget* parent = nullptr);

    void setTrajectory(const std::vector<mirror::TrajectoryPoint>& points,
                       double usableRadiusMm);
    void clearTrajectory();
    void startAnimation();
    void stopAnimation();
    void resetAnimation();
    bool isAnimationRunning() const { return m_timerId != 0; }
    void setAnimationIntervalMs(int intervalMs);

protected:
    void paintEvent(QPaintEvent* event) override;
    void timerEvent(QTimerEvent* event) override;

private:
    std::vector<mirror::TrajectoryPoint> m_points;
    double m_usableRadiusMm{330.0};
    std::size_t m_animationIndex{0};
    int m_timerId{0};
    int m_animationIntervalMs{25};
};
