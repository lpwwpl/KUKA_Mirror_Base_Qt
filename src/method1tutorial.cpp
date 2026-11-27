#include "method1tutorial.h"

#include <QHBoxLayout>
#include <QFont>
#include <QLabel>
#include <QPainter>
#include <QPainterPath>
#include <QPushButton>
#include <QTimer>
#include <QVBoxLayout>
#include <cmath>

namespace {
QString U8(const char* text) { return QString::fromUtf8(text); }

struct TutorialStepText {
    const char* title;
    const char* description;
    const char* key;
    const char* formula;
};

const TutorialStepText kSteps[] = {
    {
        "第1步：分清 TOOL、WORLD 和镜面 BASE",
        "TOOL 描述加工头相对机器人法兰的位置和方向；WORLD 是机器人固定坐标系；镜面 BASE 固定在凹镜光学顶点，随斜台面一起倾斜和移动。",
        "镜子移动时更新 BASE，不要重新标定 TOOL。",
        "TOOL：加工头自己的坐标系\nWORLD：机器人固定坐标系\nBASE：凹镜自己的坐标系"
    },
    {
        "第2步：把凹镜放到最终加工位置",
        "先把凹镜放在 -20.8° 斜台面上，再完成 Y 方向平移。BASE 只关心镜子最终在哪里、朝向哪里，不关心它是怎样移动过去的。",
        "先安装定位，后测量；不要用理论移动量代替实际顶点坐标。",
        "最终顶点：P0=(X0,Y0,Z0)\n理论姿态：A=0°, B=-20.8°, C=0°"
    },
    {
        "第3步：用已标定 TOOL 测量五个点",
        "在 $BASE=$WORLD 下测量 P0、PX+、PX-、PY+、PY-。PX±、PY±应分别处于相同半径且关于顶点对称。",
        "五点用于求真实 BASE；镜面中心只能给出原点和中心法向，不能确定完整 X/Y 方向。",
        "P0：光学顶点\nPX±：局部 X 方向对称点\nPY±：局部 Y 方向对称点"
    },
    {
        "第4步：由五点求 X_M、Y_M、Z_M 和 BASE",
        "PX+−PX- 得到镜面 X 轴。PY+−PY- 先去掉在 X 轴上的投影，再归一化得到 Y 轴。X×Y 得到朝凹面开口的 Z 轴。",
        "旋转矩阵三列就是镜面三根轴在 WORLD 中的方向。",
        "eX=normalize(PX+−PX-)\neY=normalize(vY−dot(vY,eX)eX)\neZ=normalize(eX×eY)\nR_WM=[eX eY eZ], T_WM=P0"
    },
    {
        "第5步：在镜面 BASE 下生成曲面点和逐点法向",
        "轨迹 XYZ 使用凹镜局部坐标。BASE 已包含 -20.8° 整体倾角，因此轨迹点不能再统一给 B 减 20.8°。每个点的 ABC 由该点法向单独计算。",
        "XYZ 正确不代表姿态正确；凹面边缘法向与中心法向差别很大。",
        "z=(r²/R)/(1+sqrt(1−(1+k)r²/R²))\nn=normalize(−x,−y,R−(1+k)z)"
    },
    {
        "第6步：选择轨迹并生成单文件 SRC",
        "可选择螺旋、从外向内同心圆或往复光栅。程序只生成 SRC，所有 LIN 点直接写在 SRC 中，并使用 $BASE=BASE_DATA[n]。同心圆从凹面开口侧看，每一圈都保持逆时针。",
        "从外向内同心圆先加工边缘，所有圆同向运行，再沿固定径向进入下一内圈。",
        "$TOOL=TOOL_DATA[1]\n$BASE=BASE_DATA[5]\nLIN {X xM,Y yM,Z zM,A aM,B bM,C cM} C_DIS"
    },
    {
        "第7步：动画检查和现场低速验证",
        "界面动画只验证轨迹方向、起终点和法向连续性，不等于真实机器人逆运动学仿真。上机前仍需检查 A5 奇异、A4/A6 翻转、关节限位和碰撞。",
        "先在中心、X±、Y±五个位置空运行验证，再运行完整轨迹。",
        "推荐顺序：\n计算 BASE → 生成 SRC → 动画预览 → KUKA.Sim → T1低速空跑 → 加工"
    }
};

constexpr int kStepCount = static_cast<int>(sizeof(kSteps) / sizeof(kSteps[0]));

} // namespace

class TutorialCanvas : public QWidget {
public:
    explicit TutorialCanvas(QWidget* parent = nullptr) : QWidget(parent) {
        setMinimumHeight(380);
        m_timer = new QTimer(this);
        m_timer->setInterval(35);
        QObject::connect(m_timer, &QTimer::timeout, this, [this]() {
            m_phase += 0.018;
            if (m_phase > 1.0) m_phase -= 1.0;
            update();
        });
        m_timer->start();
    }

    void setStep(int step) {
        m_step = step;
        m_phase = 0.0;
        update();
    }

protected:
    void paintEvent(QPaintEvent*) override {
        QPainter p(this);
        p.setRenderHint(QPainter::Antialiasing, true);
        p.fillRect(rect(), palette().base());

        const QRectF r = rect().adjusted(20, 20, -20, -20);
        drawWorldAxes(p, r);
        drawTableAndMirror(p, r);
        drawRobot(p, r);

        if (m_step >= 2) drawFivePoints(p, r);
        if (m_step >= 3) drawMirrorAxes(p, r);
        if (m_step >= 4) drawNormals(p, r);
        if (m_step >= 5) drawCirclePath(p, r);
        if (m_step >= 6) drawMovingTool(p, r);
    }

private:
    QPointF mirrorPoint(const QRectF& r, double x, double z) const {
        const QPointF origin(r.center().x() + 95.0, r.bottom() - 70.0);
        const double scale = std::min(r.width() / 850.0, r.height() / 460.0);
        const double beta = -20.8 * 3.14159265358979323846 / 180.0;
        const double xr = std::cos(beta) * x + std::sin(beta) * z;
        const double zr = -std::sin(beta) * x + std::cos(beta) * z;
        return QPointF(origin.x() + scale * xr, origin.y() - scale * zr);
    }

    void drawArrow(QPainter& p, const QPointF& a, const QPointF& b,
                   const QColor& color, int width = 3) const {
        p.setPen(QPen(color, width, Qt::SolidLine, Qt::RoundCap));
        p.drawLine(a, b);
        const double angle = std::atan2(b.y() - a.y(), b.x() - a.x());
        const double size = 9.0;
        QPolygonF head;
        head << b
             << QPointF(b.x() - size * std::cos(angle - 0.55), b.y() - size * std::sin(angle - 0.55))
             << QPointF(b.x() - size * std::cos(angle + 0.55), b.y() - size * std::sin(angle + 0.55));
        p.setBrush(color);
        p.drawPolygon(head);
    }

    void drawWorldAxes(QPainter& p, const QRectF& r) const {
        const QPointF o(r.left() + 55, r.bottom() - 40);
        drawArrow(p, o, o + QPointF(78, 0), QColor(190, 60, 55));
        drawArrow(p, o, o + QPointF(0, -78), QColor(55, 120, 195));
        p.setPen(palette().text().color());
        p.drawText(o + QPointF(84, 5), U8("X_W"));
        p.drawText(o + QPointF(-10, -86), U8("Z_W"));
        p.drawText(o + QPointF(-10, 24), U8("机器人 WORLD"));
    }

    void drawTableAndMirror(QPainter& p, const QRectF& r) const {
        const QPointF a = mirrorPoint(r, -330, -35);
        const QPointF b = mirrorPoint(r, 330, -35);
        p.setPen(QPen(QColor(125, 125, 125), 10, Qt::SolidLine, Qt::RoundCap));
        p.drawLine(a, b);
        p.setPen(palette().text().color());
        p.drawText((a+b)*0.5 + QPointF(-40, 34), U8("-20.8°斜台面"));

        QPainterPath mirror;
        const int samples = 80;
        for (int i = 0; i <= samples; ++i) {
            const double x = -325.0 + 650.0 * i / samples;
            const double rr = std::abs(x);
            const double R = 349.0, k = -0.60117;
            const double z = rr*rr / (R * (1.0 + std::sqrt(std::max(0.0, 1.0-(1.0+k)*rr*rr/(R*R)))));
            const QPointF q = mirrorPoint(r, x, z);
            if (i == 0) mirror.moveTo(q); else mirror.lineTo(q);
        }
        p.setPen(QPen(QColor(45, 105, 175), 8, Qt::SolidLine, Qt::RoundCap));
        p.drawPath(mirror);

        const QPointF vertex = mirrorPoint(r, 0.0, 0.0);
        const QPointF opening = mirrorPoint(r, 0.0, 82.0);
        drawArrow(p, vertex, opening, QColor(70,155,90), 3);

        p.setPen(palette().text().color());
        p.drawText(mirrorPoint(r, -30, 5) + QPointF(-45, 42), U8("凹非球面镜"));
        p.drawText(opening + QPointF(-105, -8), U8("凹面开口朝机器人"));
    }

    void drawRobot(QPainter& p, const QRectF& r) const {
        // The robot is drawn on the opening side of the concave mirror.
        // The centre tool axis is collinear with +Z_M and points from the
        // robot wrist toward the mirror vertex.
        const QPointF mirrorVertex = mirrorPoint(r, 0.0, 0.0);
        const QPointF openingPoint = mirrorPoint(r, 0.0, 135.0);
        QPointF openingVector = openingPoint - mirrorVertex;
        const double openingLength = std::hypot(openingVector.x(), openingVector.y());
        if (openingLength > 1e-9) openingVector /= openingLength;

        const QPointF wrist = mirrorVertex + 125.0 * openingVector;
        const QPointF base(r.left() + 105.0, r.bottom() - 42.0);
        const QPointF elbow(base.x() + 72.0, base.y() - 105.0);

        p.setPen(Qt::NoPen);
        p.setBrush(QColor(235, 150, 35));
        p.drawRoundedRect(QRectF(base.x()-45, base.y()-35, 90, 55), 7, 7);
        p.setPen(QPen(QColor(235, 150, 35), 16, Qt::SolidLine, Qt::RoundCap));
        p.drawLine(base + QPointF(0,-30), elbow);
        p.drawLine(elbow, wrist);
        p.setBrush(QColor(250, 190, 70));
        p.setPen(Qt::NoPen);
        p.drawEllipse(elbow, 12, 12);
        p.drawEllipse(wrist, 10, 10);

        p.setPen(QPen(QColor(70,70,70), 8, Qt::SolidLine, Qt::RoundCap));
        p.drawLine(wrist, mirrorVertex);
        p.setBrush(QColor(210,65,65));
        p.setPen(Qt::NoPen);
        p.drawEllipse(mirrorVertex, 5, 5);

        p.setPen(palette().text().color());
        p.drawText(base + QPointF(-78, 40), U8("KUKA KR 70 R2100"));
        p.drawText(wrist + QPointF(-88, -16), U8("工具从凹面开口侧接近"));
    }

    void drawFivePoints(QPainter& p, const QRectF& r) const {
        struct PointData { double x; double z; const char* name; };
        const double sr = 220.0;
        const double R = 349.0, k = -0.60117;
        const double sz = sr*sr/(R*(1+std::sqrt(1-(1+k)*sr*sr/(R*R))));
        const PointData pts[] = {{0,0,"P0"},{sr,sz,"PX+"},{-sr,sz,"PX-"}};
        p.setFont(QFont(p.font().family(), 9));
        for (const auto& d : pts) {
            const QPointF q = mirrorPoint(r,d.x,d.z);
            p.setBrush(QColor(210,65,65)); p.setPen(Qt::NoPen); p.drawEllipse(q,6,6);
            p.setPen(palette().text().color()); p.drawText(q+QPointF(7,-7),U8(d.name));
        }
        const QPointF q0=mirrorPoint(r,0,0);
        p.setPen(QPen(QColor(65,155,90),2,Qt::DashLine));
        p.drawEllipse(q0,54.0,20.0);
        p.setPen(palette().text().color());
        p.drawText(q0+QPointF(-75,-28),U8("PY±位于垂直纸面的对称方向"));
    }

    void drawMirrorAxes(QPainter& p, const QRectF& r) const {
        const QPointF o=mirrorPoint(r,0,0);
        const QPointF ex=mirrorPoint(r,110,0);
        const QPointF ez=mirrorPoint(r,0,115);
        drawArrow(p,o,ex,QColor(190,60,55),4);
        drawArrow(p,o,ez,QColor(55,120,195),4);
        p.setPen(palette().text().color());
        p.drawText(ex+QPointF(8,4),U8("X_M"));
        p.drawText(ez+QPointF(6,-4),U8("Z_M（朝凹面开口）"));
        p.drawText(o+QPointF(-20,24),U8("BASE原点 P0"));
    }

    void drawNormals(QPainter& p, const QRectF& r) const {
        const double xs[] = {-250,-120,0,120,250};
        const double R=349.0,k=-0.60117;
        for(double x:xs){
            const double rr=std::abs(x);
            const double z=rr*rr/(R*(1+std::sqrt(1-(1+k)*rr*rr/(R*R))));
            double nx=-x, nz=R-(1+k)*z;
            const double l=std::hypot(nx,nz); nx/=l; nz/=l;
            const QPointF q=mirrorPoint(r,x,z);
            const QPointF tip=mirrorPoint(r,x+55*nx,z+55*nz);
            drawArrow(p,q,tip,QColor(70,155,90),2);
        }
        p.setPen(palette().text().color());
        p.drawText(r.left()+190,r.top()+36,U8("每个加工点都有不同法向，ABC必须逐点变化"));
    }

    void drawCirclePath(QPainter& p, const QRectF& r) const {
        const QPointF o=mirrorPoint(r,0,20);
        p.save();
        p.translate(o);
        p.rotate(20.8);
        p.setPen(QPen(QColor(135,70,180),2,Qt::DashLine));
        for(int radius=125;radius>=20;radius-=22) {
            p.drawEllipse(QPointF(0,0),radius,0.38*radius);

            // All arrows use increasing mathematical angle. With the painter's
            // downward screen Y axis, the minus sign keeps the displayed motion
            // counter-clockwise when viewed from +Z_M (the opening side).
            const double t1=0.06;
            const double t2=0.30;
            const QPointF a(radius*std::cos(t1), -0.38*radius*std::sin(t1));
            const QPointF b(radius*std::cos(t2), -0.38*radius*std::sin(t2));
            drawArrow(p,a,b,QColor(135,70,180),2);
        }
        p.setPen(QPen(QColor(135,70,180),2,Qt::SolidLine));
        p.drawLine(QPointF(125,0),QPointF(20,0));
        p.restore();
        p.setPen(palette().text().color());
        p.drawText(r.left()+200,r.top()+62,U8("从 +Z_M 开口侧看：所有圆均逆时针，逐圈向中心收缩"));
    }

    void drawMovingTool(QPainter& p, const QRectF& r) const {
        // Schematic outside-in concentric path. Every ring uses the same
        // positive angular direction; a short connector at angle zero moves
        // to the next inner ring.
        const double radii[] = {260.0, 215.0, 170.0, 125.0, 80.0, 35.0, 0.0};
        constexpr int ringCount = 6;
        const double connectorUnits = 0.18;
        const double totalUnits = ringCount * (1.0 + connectorUnits);
        double u = m_phase * totalUnits;
        double radius = radii[0];
        double angle = 0.0;

        for (int ring = 0; ring < ringCount; ++ring) {
            if (u <= 1.0) {
                radius = radii[ring];
                angle = 2.0 * 3.14159265358979323846 * u; // always CCW from +Z_M
                break;
            }
            u -= 1.0;
            if (u <= connectorUnits) {
                const double fraction = u / connectorUnits;
                radius = radii[ring] + (radii[ring+1] - radii[ring]) * fraction;
                angle = 0.0;
                break;
            }
            u -= connectorUnits;
        }

        const double x=radius*std::cos(angle);
        const double y=radius*std::sin(angle);
        const double rr=std::hypot(x,y);
        const double R=349.0,k=-0.60117;
        const double z=rr*rr/(R*(1+std::sqrt(std::max(0.0,1-(1+k)*rr*rr/(R*R)))));
        double nx=-x, ny=-y, nz=R-(1+k)*z;
        const double len=std::sqrt(nx*nx+ny*ny+nz*nz); nx/=len; ny/=len; nz/=len;
        const QPointF q=mirrorPoint(r,x,z);
        const QPointF tip=mirrorPoint(r,x+75*nx,z+75*nz);
        p.setBrush(QColor(240,70,55)); p.setPen(Qt::NoPen); p.drawEllipse(q,7,7);
        drawArrow(p,q,tip,QColor(240,70,55),4);
        p.setPen(palette().text().color());
        p.drawText(r.left()+200,r.top()+88,U8("红点：外圈到内圈；所有圆同向；红箭头：凹面开口法向"));
    }

    int m_step{0};
    double m_phase{0.0};
    QTimer* m_timer{nullptr};
};


Method1TutorialWidget::Method1TutorialWidget(QWidget* parent) : QWidget(parent) {
    auto* root = new QVBoxLayout(this);
    m_canvas = new TutorialCanvas;
    root->addWidget(m_canvas, 3);

    auto* controls = new QHBoxLayout;
    auto* previous = new QPushButton(U8("上一步"));
    m_playButton = new QPushButton(U8("自动播放"));
    auto* next = new QPushButton(U8("下一步"));
    m_stepLabel = new QLabel;
    controls->addWidget(previous);
    controls->addWidget(m_playButton);
    controls->addWidget(next);
    controls->addStretch();
    controls->addWidget(m_stepLabel);
    root->addLayout(controls);

    m_titleLabel = new QLabel;
    QFont titleFont = m_titleLabel->font();
    titleFont.setPointSize(titleFont.pointSize()+2);
    titleFont.setBold(true);
    m_titleLabel->setFont(titleFont);
    m_descriptionLabel = new QLabel;
    m_descriptionLabel->setWordWrap(true);
    m_keyLabel = new QLabel;
    m_keyLabel->setWordWrap(true);
    m_keyLabel->setStyleSheet(U8("QLabel { background:#eef6ff; border:1px solid #a7c9ee; padding:8px; border-radius:5px; }"));
    m_formulaLabel = new QLabel;
    m_formulaLabel->setTextInteractionFlags(Qt::TextSelectableByMouse);
    m_formulaLabel->setStyleSheet(U8("QLabel { font-family:Consolas,'Courier New'; background:#f5f5f5; border:1px solid #cccccc; padding:8px; }"));
    root->addWidget(m_titleLabel);
    root->addWidget(m_descriptionLabel);
    root->addWidget(m_keyLabel);
    root->addWidget(m_formulaLabel);

    m_timer = new QTimer(this);
    m_timer->setInterval(3200);
    connect(m_timer, &QTimer::timeout, this, [this](){ nextStep(); });
    connect(previous, &QPushButton::clicked, this, [this](){ previousStep(); });
    connect(next, &QPushButton::clicked, this, [this](){ nextStep(); });
    connect(m_playButton, &QPushButton::clicked, this, [this](){ togglePlay(); });
    setStep(0);
}

void Method1TutorialWidget::setStep(int step) {
    m_step = (step % kStepCount + kStepCount) % kStepCount;
    m_canvas->setStep(m_step);
    updateTexts();
}
void Method1TutorialWidget::nextStep(){ setStep(m_step+1); }
void Method1TutorialWidget::previousStep(){ setStep(m_step-1); }
void Method1TutorialWidget::togglePlay(){
    if(m_timer->isActive()){
        m_timer->stop();
        m_playButton->setText(U8("自动播放"));
    }else{
        m_timer->start();
        m_playButton->setText(U8("暂停"));
    }
}
void Method1TutorialWidget::updateTexts(){
    const TutorialStepText& s=kSteps[m_step];
    m_stepLabel->setText(U8("步骤 %1 / %2").arg(m_step+1).arg(kStepCount));
    m_titleLabel->setText(U8(s.title));
    m_descriptionLabel->setText(U8(s.description));
    m_keyLabel->setText(U8("关键：")+U8(s.key));
    m_formulaLabel->setText(U8(s.formula));
}
