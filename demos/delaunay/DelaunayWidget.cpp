#include "DelaunayWidget.hpp"

#include "data_structures/DoublyConnectedEdgeListAlgorithms.hpp"
#include "delaunay/DelaunayTriangulator.hpp"
#include <QPainter>
#include <QWheelEvent>
#include <algorithm>
#include <cmath>

namespace {
    constexpr QSize MinimumWidgetSize{520, 420};
    constexpr double PlotMargin = 32.0;
    constexpr QRectF WorldBounds{0.0, 0.0, 100.0, 100.0};
    constexpr qreal MinimumZoom = 0.2;
    constexpr qreal MaximumZoom = 8.0;
    constexpr qreal ZoomStep = 1.15;
    constexpr qreal WheelDeltaPerStep = 120.0;

    constexpr double TriangulationPenWidth = 0.8;
    constexpr QColor BackgroundColor{248, 250, 252};
    constexpr QColor TriangulationColor{15, 118, 110};
} // namespace

DelaunayWidget::DelaunayWidget(QWidget* parent)
    : QWidget{parent} {
    setMinimumSize(MinimumWidgetSize);
    setMouseTracking(true);
    setAutoFillBackground(false);
}

void DelaunayWidget::clear() {
    Vertices.clear();
    Triangulation = {};
    update();
}

QRectF DelaunayWidget::GetPlotRectangle() const {
    return {PlotMargin, PlotMargin, width() - 2.0 * PlotMargin, height() - 2.0 * PlotMargin};
}

QPointF DelaunayWidget::ConvertToWorld(const QPointF& widgetPosition) const {
    const auto plot = GetPlotRectangle();
    const auto scaleX = plot.width() / WorldBounds.width() * Zoom;
    const auto scaleY = plot.height() / WorldBounds.height() * Zoom;
    const auto x = ViewCenter.x() + (widgetPosition.x() - plot.left()) / scaleX;
    const auto y = ViewCenter.y() - (widgetPosition.y() - plot.bottom()) / scaleY;
    return {x, y};
}

void DelaunayWidget::mousePressEvent(QMouseEvent* event) {
    if (event->button() == Qt::RightButton) {
        IsPanning = true;
        LastPanPosition = event->position();
        setCursor(Qt::ClosedHandCursor);
    } else if (event->button() == Qt::LeftButton) {
        const auto position = ConvertToWorld(event->position());
        const compg::Vertex2D vertex{position.x(), position.y()};
        if (std::ranges::find(Vertices, vertex) == Vertices.end()) {
            Vertices.push_back(vertex);
            UpdateTriangulation();
        }
        emit clicked(position);
    }
    QWidget::mousePressEvent(event);
}

void DelaunayWidget::mouseMoveEvent(QMouseEvent* event) {
    if (IsPanning) {
        const auto delta = event->position() - LastPanPosition;
        const auto plot = GetPlotRectangle();
        ViewCenter.rx() -= delta.x() / (plot.width() / WorldBounds.width() * Zoom);
        ViewCenter.ry() += delta.y() / (plot.height() / WorldBounds.height() * Zoom);
        LastPanPosition = event->position();
        update();
    }
    QWidget::mouseMoveEvent(event);
}

void DelaunayWidget::mouseReleaseEvent(QMouseEvent* event) {
    if (event->button() == Qt::RightButton) {
        IsPanning = false;
        unsetCursor();
    }
    QWidget::mouseReleaseEvent(event);
}

void DelaunayWidget::UpdateTriangulation() {
    Triangulation = Triangulator.Triangulate(Vertices);
    update();
}

void DelaunayWidget::paintEvent(QPaintEvent*) {
    QPainter painter{this};
    painter.setRenderHint(QPainter::Antialiasing);
    painter.fillRect(rect(), BackgroundColor);

    const auto plot = GetPlotRectangle();
    painter.save();
    painter.translate(plot.left(), plot.bottom());
    painter.scale(plot.width() / WorldBounds.width() * Zoom, -plot.height() / WorldBounds.height() * Zoom);
    painter.translate(-ViewCenter.x(), -ViewCenter.y());

    QPen triangulationPen{TriangulationColor, TriangulationPenWidth};
    triangulationPen.setCosmetic(true);
    painter.setPen(triangulationPen);
    compg::WalkUndirectedEdges(Triangulation, [this, &painter](auto edgeIndex) {
        const auto segment = Triangulation.GetEdgeAsLineSegment(edgeIndex);
        painter.drawLine(QLineF{segment[0][0], segment[0][1], segment[1][0], segment[1][1]});
    });
    painter.restore();
}

void DelaunayWidget::wheelEvent(QWheelEvent* event) {
    const auto delta = event->angleDelta().y();
    if (delta == 0) {
        event->ignore();
        return;
    }

    const auto cursorWorldPosition = ConvertToWorld(event->position());
    const auto zoomFactor = std::pow(ZoomStep, static_cast<qreal>(delta) / WheelDeltaPerStep);
    Zoom = std::clamp(Zoom * zoomFactor, MinimumZoom, MaximumZoom);
    const auto newCursorWorldPosition = ConvertToWorld(event->position());
    ViewCenter += cursorWorldPosition - newCursorWorldPosition;
    update();
    event->accept();
}
