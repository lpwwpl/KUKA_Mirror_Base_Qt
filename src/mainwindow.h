#pragma once

#include "mirrorcore.h"

#include <QMainWindow>
#include <optional>

class QCheckBox;
class QComboBox;
class QDoubleSpinBox;
class QLineEdit;
class QPlainTextEdit;
class QPushButton;
class QSpinBox;
class QTableWidget;
class QLabel;
class PathPreviewWidget;

class MainWindow : public QMainWindow {
public:
    explicit MainWindow(QWidget* parent = nullptr);

private:
    QWidget* createTutorialTab();
    QWidget* createBaseTab();
    QWidget* createTrajectoryTab();
    QWidget* createOutputTab();

    QDoubleSpinBox* createDoubleSpin(double minimum, double maximum,
                                     double value, int decimals,
                                     double step = 1.0) const;
    QSpinBox* createIntSpin(int minimum, int maximum, int value) const;

    mirror::FivePoints readFivePoints() const;
    mirror::MirrorParameters readMirrorParameters() const;
    mirror::TrajectoryOptions readTrajectoryOptions() const;
    mirror::KrlOptions readKrlOptions() const;

    void loadExamplePoints();
    void calculateBase();
    void generateProgram();
    void importPointsCsv();
    void exportPointsCsv();
    void exportGeneratedSrc();
    void copySrcCode();
    void updateBaseDisplays(const mirror::RealBase& base,
                            const mirror::Diagnostics& diagnostics);
    void updateOutputDisplays(const mirror::GeneratedOutput& output);
    void showError(const QString& message);

    QTableWidget* m_pointsTable{nullptr};
    QPlainTextEdit* m_baseResult{nullptr};
    QPlainTextEdit* m_diagnosticsResult{nullptr};

    QDoubleSpinBox* m_diameter{nullptr};
    QDoubleSpinBox* m_radius{nullptr};
    QDoubleSpinBox* m_conicK{nullptr};
    QComboBox* m_pathType{nullptr};
    QDoubleSpinBox* m_edgeMargin{nullptr};
    QDoubleSpinBox* m_pathPitch{nullptr};
    QDoubleSpinBox* m_pointSpacing{nullptr};
    QDoubleSpinBox* m_normalOffset{nullptr};
    QDoubleSpinBox* m_clearance{nullptr};
    QDoubleSpinBox* m_rollOffset{nullptr};
    QComboBox* m_pathDirection{nullptr};
    QComboBox* m_toolZMode{nullptr};
    QComboBox* m_orientationReference{nullptr};
    QSpinBox* m_maxPoints{nullptr};

    QLineEdit* m_programName{nullptr};
    QSpinBox* m_toolIndex{nullptr};
    QSpinBox* m_baseIndex{nullptr};
    QDoubleSpinBox* m_velocity{nullptr};
    QDoubleSpinBox* m_acceleration{nullptr};
    QDoubleSpinBox* m_cdis{nullptr};
    QCheckBox* m_useCdis{nullptr};
    QCheckBox* m_writeBaseRuntime{nullptr};

    QLabel* m_summaryLabel{nullptr};
    QPushButton* m_simulationButton{nullptr};
    PathPreviewWidget* m_pathPreview{nullptr};
    QPlainTextEdit* m_srcPreview{nullptr};

    std::optional<mirror::RealBase> m_lastBase;
    std::optional<mirror::Diagnostics> m_lastDiagnostics;
    std::optional<mirror::GeneratedOutput> m_lastOutput;
};
