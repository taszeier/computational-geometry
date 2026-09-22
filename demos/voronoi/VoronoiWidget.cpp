#include "VoronoiWidget.hpp"

#include <QMouseEvent>
#include <QPainter>

#include <algorithm>

#include "data_structures/DoublyConnectedEdgeListAlgorithms.hpp"
#include "math/primitives/Box.hpp"

namespace {
    constexpr QSize MinimumWidgetSize{520, 420};
    constexpr double PlotMargin = 32.0;
    constexpr QRectF WorldBounds{0.0, 0.0, 100.0, 100.0};
    const compg::Box2D CalculatorBounds{{-50.0, -50.0}, {150.0, 150.0}};

    constexpr QColor BackgroundColor{248, 250, 252};
    constexpr QColor DiagramColor{15, 118, 110};
    constexpr QColor SiteColor{220, 38, 38};
    constexpr QColor BorderColor{100, 116, 139};

    constexpr double DiagramPenWidth = 0.8;
    constexpr double BorderPenWidth = 1.0;
    constexpr double SiteRadius = 3.0;
} // namespace

VoronoiWidget::VoronoiWidget(QWidget* parent)
    : QWidget{parent} {
    setMinimumSize(MinimumWidgetSize);
    setMouseTracking(true);
    setAutoFillBackground(false);
}

void VoronoiWidget::clear() {
    Sites.clear();
    Diagram = {};
    update();
}

QRectF VoronoiWidget::GetPlotRectangle() const {
    return {PlotMargin, PlotMargin, width() - 2.0 * PlotMargin, height() - 2.0 * PlotMargin};
}

QPointF VoronoiWidget::ConvertToWorld(const QPointF& widgetPosition) const {
    const auto plot = GetPlotRectangle();
    const auto scaleX = plot.width() / WorldBounds.width();
    const auto scaleY = plot.height() / WorldBounds.height();
    const auto x = ViewCenter.x() + (widgetPosition.x() - plot.center().x()) / scaleX;
    const auto y = ViewCenter.y() - (widgetPosition.y() - plot.center().y()) / scaleY;
    return {x, y};
}

void VoronoiWidget::mousePressEvent(QMouseEvent* event) {
    if (event->button() != Qt::LeftButton || !GetPlotRectangle().contains(event->position())) {
        event->ignore();
        return;
    }

    const auto position = ConvertToWorld(event->position());
    const compg::Vertex2D site{position.x(), position.y()};
    if (std::ranges::find(Sites, site) == Sites.end()) {
        Sites.push_back(site);
        UpdateDiagram();
    }
    emit clicked(position);
    event->accept();
}

void VoronoiWidget::UpdateDiagram() {
    Diagram = Calculator.FindVoronoiDiagram(Sites, CalculatorBounds);
    update();
}

void VoronoiWidget::paintEvent(QPaintEvent*) {
    QPainter painter{this};
    painter.setRenderHint(QPainter::Antialiasing);
    painter.fillRect(rect(), BackgroundColor);

    const auto plot = GetPlotRectangle();
    const auto scaleX = plot.width() / WorldBounds.width();
    const auto scaleY = plot.height() / WorldBounds.height();
    painter.save();
    painter.setClipRect(plot);
    painter.translate(plot.center());
    painter.scale(scaleX, -scaleY);
    painter.translate(-ViewCenter.x(), -ViewCenter.y());

    QPen diagramPen{DiagramColor, DiagramPenWidth};
    diagramPen.setCosmetic(true);
    painter.setPen(diagramPen);

    compg::WalkUndirectedEdges(Diagram, [&painter, this](auto edgeIndex) {
        const auto segment = Diagram.GetEdgeAsLineSegment(edgeIndex);
        painter.drawLine(QLineF{segment[0][0], segment[0][1], segment[1][0], segment[1][1]});
    });

    painter.setPen(Qt::NoPen);
    painter.setBrush(SiteColor);
    for (const auto& site : Sites) {
        painter.drawEllipse(QPointF{site[0], site[1]}, SiteRadius / scaleX, SiteRadius / scaleY);
    }
    painter.restore();

    painter.setPen(QPen{BorderColor, BorderPenWidth});
    painter.setBrush(Qt::NoBrush);
    painter.drawRect(plot);
}