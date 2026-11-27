#include "pathpreview.h"

#include <QPainter>
#include <QPainterPath>
#include <QTimerEvent>
#include <algorithm>
#include <cmath>

namespace {
QString U8(const char* text) { return QString::fromUtf8(text); }
}

PathPreviewWidget::PathPreviewWidget(QWidget* parent)
    : QWidget(parent) {
    setMinimumSize(360, 320);
}

void PathPreviewWidget::setTrajectory(const std::vector<mirror::TrajectoryPoint>& points,
                                      double usableRadiusMm) {
    stopAnimation();
    m_points = points;
    m_usableRadiusMm = std::max(1.0, usableRadiusMm);
    m_animationIndex = 0;
    update();
}

void PathPreviewWidget::clearTrajectory() {
    stopAnimation();
    m_points.clear();
    m_animationIndex = 0;
    update();
}

void PathPreviewWidget::startAnimation() {
    if (m_points.empty() || m_timerId != 0) return;
    m_timerId = startTimer(m_animationIntervalMs);
}

void PathPreviewWidget::stopAnimation() {
    if (m_timerId != 0) {
        killTimer(m_timerId);
        m_timerId = 0;
    }
}

void PathPreviewWidget::resetAnimation() {
    m_animationIndex = 0;
    update();
}

void PathPreviewWidget::setAnimationIntervalMs(int intervalMs) {
    m_animationIntervalMs = std::max(5, intervalMs);
    if (m_timerId != 0) {
        stopAnimation();
        startAnimation();
    }
}

void PathPreviewWidget::timerEvent(QTimerEvent* event) {
    if (event->timerId() != m_timerId || m_points.empty()) return;
    const std::size_t step = std::max<std::size_t>(1, m_points.size() / 1600);
    m_animationIndex += step;
    if (m_animationIndex >= m_points.size()) m_animationIndex = 0;
    update();
}

void PathPreviewWidget::paintEvent(QPaintEvent*) {
    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing, true);
    painter.fillRect(rect(), palette().base());

    const QRectF area = rect().adjusted(28, 28, -28, -44);
    const QPointF center = area.center();
    const double scale = 0.48 * std::min(area.width(), area.height()) / m_usableRadiusMm;

    painter.setPen(QPen(QColor(120, 120, 120), 1));
    painter.setBrush(QColor(245, 248, 252));
    painter.drawEllipse(center, m_usableRadiusMm * scale, m_usableRadiusMm * scale);

    painter.setPen(QPen(QColor(185, 185, 185), 1, Qt::DashLine));
    painter.drawLine(QPointF(center.x() - m_usableRadiusMm*scale, center.y()),
                     QPointF(center.x() + m_usableRadiusMm*scale, center.y()));
    painter.drawLine(QPointF(center.x(), center.y() - m_usableRadiusMm*scale),
                     QPointF(center.x(), center.y() + m_usableRadiusMm*scale));

    auto mapPoint = [&](const mirror::Vec3& p) {
        return QPointF(center.x() + p.x * scale, center.y() - p.y * scale);
    };

    if (!m_points.empty()) {
        QPainterPath path;
        path.moveTo(mapPoint(m_points.front().pointM));
        for (std::size_t i = 1; i < m_points.size(); ++i) {
            path.lineTo(mapPoint(m_points[i].pointM));
        }
        painter.setPen(QPen(QColor(26, 99, 190), 1.4));
        painter.setBrush(Qt::NoBrush);
        painter.drawPath(path);

        painter.setBrush(QColor(34, 155, 86));
        painter.setPen(Qt::NoPen);
        painter.drawEllipse(mapPoint(m_points.front().pointM), 4, 4);
        painter.setBrush(QColor(202, 64, 64));
        painter.drawEllipse(mapPoint(m_points.back().pointM), 4, 4);

        const std::size_t idx = std::min(m_animationIndex, m_points.size()-1);
        const mirror::TrajectoryPoint& current = m_points[idx];
        const QPointF q = mapPoint(current.pointM);
        const double normalVisualMm = std::max(20.0, 0.18 * m_usableRadiusMm);
        const QPointF normalTip = mapPoint(current.pointM + normalVisualMm * current.openNormalM);
        painter.setPen(QPen(QColor(220, 70, 45), 2.5, Qt::SolidLine, Qt::RoundCap));
        painter.drawLine(q, normalTip);
        painter.setBrush(QColor(235, 70, 45));
        painter.setPen(Qt::NoPen);
        painter.drawEllipse(q, 6, 6);
        painter.setPen(palette().text().color());
        painter.drawText(QRectF(8, 4, width()-16, 22), Qt::AlignCenter,
                         U8("仿真点 %1 / %2；红线为凹面开口法向")
                             .arg(idx+1).arg(m_points.size()));
    }

    painter.setPen(palette().text().color());
    painter.drawText(QRectF(8, height() - 34, width() - 16, 26),
                     Qt::AlignCenter,
                     m_points.empty()
                         ? U8("生成轨迹后显示局部 X-Y 平面预览")
                         : U8("绿色：起点  红色：终点  蓝线：轨迹  红点/红线：动画位置与法向"));
}
