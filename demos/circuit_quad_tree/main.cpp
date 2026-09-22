#include "CircuitQuadTreeWidget.hpp"

#include <QApplication>
#include <QFormLayout>
#include <QLabel>
#include <QMainWindow>
#include <QPushButton>
#include <QShortcut>
#include <QSpinBox>
#include <QStatusBar>
#include <QVBoxLayout>

int main(int argc, char* argv[]) {
    QApplication application{argc, argv};

    QMainWindow window;
    window.setWindowTitle("Computational Geometry - Circuit Quad Tree");

    auto* content = new QWidget{&window};
    auto* layout = new QVBoxLayout{content};
    layout->setContentsMargins(12, 12, 12, 12);
    layout->setSpacing(8);

    auto* description
        = new QLabel{"Circuit quad tree playground: draw integer segments at 0, 45, 90, or 135 degrees.", content};
    description->setStyleSheet("font-size: 14px; font-weight: 600; color: #1e293b;");

    auto* controls = new QWidget{content};
    auto* controlsLayout = new QHBoxLayout{controls};
    controlsLayout->setContentsMargins(0, 0, 0, 0);
    auto* powerLabel = new QLabel{"power:", controls};
    auto* power = new QSpinBox{controls};
    power->setRange(1, 10);
    power->setValue(5);
    power->setToolTip("The circuit box is [0, 2^power] x [0, 2^power]");
    auto* clearButton = new QPushButton{"Clear", controls};
    controlsLayout->addWidget(powerLabel);
    controlsLayout->addWidget(power);
    controlsLayout->addSpacing(12);
    controlsLayout->addWidget(clearButton);
    controlsLayout->addStretch();

    auto* shortcuts = new QLabel{"Left drag: add segment    Right drag: pan    Wheel: zoom    Ctrl+Q: quit", content};
    shortcuts->setStyleSheet("color: #475569;");

    auto* canvas = new CircuitQuadTreeWidget{content};
    layout->addWidget(description);
    layout->addWidget(controls);
    layout->addWidget(shortcuts);
    layout->addWidget(canvas, 1);
    window.setCentralWidget(content);
    window.statusBar()->showMessage("Click and drag to add an integer circuit segment");

    QShortcut quitShortcut{QKeySequence{Qt::CTRL | Qt::Key_Q}, &window};
    QObject::connect(&quitShortcut, &QShortcut::activated, &application, &QApplication::quit);
    QObject::connect(power, &QSpinBox::valueChanged, canvas, &CircuitQuadTreeWidget::SetPower);
    QObject::connect(clearButton, &QPushButton::clicked, canvas, &CircuitQuadTreeWidget::clear);
    QObject::connect(
        canvas, &CircuitQuadTreeWidget::messageChanged, &window,
        [statusBar = window.statusBar()](const QString& message) { statusBar->showMessage(message); }
    );

    window.resize(860, 720);
    window.show();
    return application.exec();
}