#pragma once

#include <QWidget>

class QLabel;
class QPushButton;
class QTimer;
class TutorialCanvas;

class Method1TutorialWidget : public QWidget {
public:
    explicit Method1TutorialWidget(QWidget* parent = nullptr);

private:
    void setStep(int step);
    void nextStep();
    void previousStep();
    void togglePlay();
    void updateTexts();

    TutorialCanvas* m_canvas{nullptr};
    QLabel* m_stepLabel{nullptr};
    QLabel* m_titleLabel{nullptr};
    QLabel* m_descriptionLabel{nullptr};
    QLabel* m_keyLabel{nullptr};
    QLabel* m_formulaLabel{nullptr};
    QPushButton* m_playButton{nullptr};
    QTimer* m_timer{nullptr};
    int m_step{0};
};
