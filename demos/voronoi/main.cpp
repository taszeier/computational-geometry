#include "VoronoiWidget.hpp"

#include <QApplication>
#include <QLabel>
#include <QMainWindow>
#include <QShortcut>
#include <QStatusBar>
#include <QVBoxLayout>

int main(int argc, char* argv[]) {
    QApplication application{argc, argv};

    QMainWindow window;
    window.setWindowTitle("Computational Geometry - Voronoi Diagram");

    auto* content = new QWidget{&window};
    auto* layout = new QVBoxLayout{content};
    layout->setContentsMargins(12, 12, 12, 12);
    layout->setSpacing(8);

    auto* description = new QLabel{
        "Voronoi diagram playground: click to add sites and inspect their nearest-site regions.", content
    };
    description->setStyleSheet("font-size: 14px; font-weight: 600; color: #1e293b;");

    auto* shortcuts = new QLabel{
        "Left click: add site    Ctrl+C: clear    Ctrl+Q: quit", content
    };
    shortcuts->setStyleSheet("color: #475569;");

    auto* canvas = new VoronoiWidget{content};
    layout->addWidget(description);
    layout->addWidget(shortcuts);
    layout->addWidget(canvas, 1);
    window.setCentralWidget(content);
    window.statusBar()->showMessage("Click the drawing to add a site");

    QShortcut quitShortcut{QKeySequence{Qt::CTRL | Qt::Key_Q}, &window};
    QObject::connect(&quitShortcut, &QShortcut::activated, &application, &QApplication::quit);

    QShortcut clearShortcut{QKeySequence{Qt::CTRL | Qt::Key_C}, &window};
    QObject::connect(&clearShortcut, &QShortcut::activated, canvas, &VoronoiWidget::clear);

    QObject::connect(canvas, &VoronoiWidget::clicked, &window, [&window](const QPointF& position) {
        window.statusBar()->showMessage(
            QString{"Site at (%1, %2)"}.arg(position.x(), 0, 'f', 2).arg(position.y(), 0, 'f', 2)
        );
    });

    window.resize(720, 600);
    window.show();
    return application.exec();
}