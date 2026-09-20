#include "ConvexHullWidget.hpp"

#include <QMouseEvent>
#include <QPainter>
#include <QWheelEvent>

#include <algorithm>
#include <ranges>


namespace {
    constexpr QSize MinimumWidgetSize{520, 420};
    constexpr double PlotMargin = 32.0;
    constexpr QRectF WorldBounds{0.0, 0.0, 100.0, 100.0};
    constexpr qreal MinimumZoom = 0.25;
    constexpr qreal MaximumZoom = 8.0;
    constexpr qreal ZoomStep = 1.15;
    constexpr qreal WheelDeltaPerStep = 120.0;

    constexpr QColor BackgroundColor{248, 250, 252};
    constexpr QColor FillColor{14, 116, 144, 45};
    constexpr QColor HullColor{14, 116, 144};
    constexpr QColor PointColor{220, 38, 38};
    constexpr QColor BorderColor{100, 116, 139};

    constexpr qreal HullPenWidth = 1.5;
    constexpr qreal BorderPenWidth = 1.0;
    constexpr double PointRadius = 3.0;
}

ConvexHullWidget::ConvexHullWidget(QWidget* parent)
    : QWidget{parent} {
    setMinimumSize(MinimumWidgetSize);
    setAutoFillBackground(false);
}

void ConvexHullWidget::clear() {
    Points.clear();
    Hull = {};
    update();
}

QRectF ConvexHullWidget::GetPlotRectangle() const {
    return {PlotMargin, PlotMargin, width() - 2.0 * PlotMargin, height() - 2.0 * PlotMargin};
}

QPointF ConvexHullWidget::ConvertToWorld(const QPointF& widgetPosition) const {
    const auto plot = GetPlotRectangle();
    const auto scaleX = plot.width() / (WorldBounds.width() / Zoom);
    const auto scaleY = plot.height() / (WorldBounds.height() / Zoom);
    const auto x = ViewCenter.x() + (widgetPosition.x() - plot.center().x()) / scaleX;
    const auto y = ViewCenter.y() - (widgetPosition.y() - plot.center().y()) / scaleY;
    return {x, y};
}

void ConvexHullWidget::mousePressEvent(QMouseEvent* event) {
    if (event->button() == Qt::RightButton && GetPlotRectangle().contains(event->position())) {
        IsPanning = true;
        LastPanPosition = event->position();
        setCursor(Qt::ClosedHandCursor);
        event->accept();
        return;
    }

    if (event->button() != Qt::LeftButton || !GetPlotRectangle().contains(event->position())) {
        event->ignore();
        return;
    }

    const auto position = ConvertToWorld(event->position());
    const compg::Vertex2D point{position.x(), position.y()};
    if (std::ranges::find(Points, point) == Points.end()) {
        Points.push_back(point);
        updateHull();
    }
    emit clicked(position);
    event->accept();
}

void ConvexHullWidget::mouseMoveEvent(QMouseEvent* event) {
    if (!IsPanning) {
        event->ignore();
        return;
    }

    const auto position = event->position();
    const auto delta = position - LastPanPosition;
    const auto plot = GetPlotRectangle();
    const auto scaleX = plot.width() / (WorldBounds.width() / Zoom);
    const auto scaleY = plot.height() / (WorldBounds.height() / Zoom);
    ViewCenter.rx() -= delta.x() / scaleX;
    ViewCenter.ry() += delta.y() / scaleY;
    LastPanPosition = position;
    update();
    event->accept();
}

void ConvexHullWidget::mouseReleaseEvent(QMouseEvent* event) {
    if (event->button() != Qt::RightButton || !IsPanning) {
        event->ignore();
        return;
    }

    IsPanning = false;
    unsetCursor();
    event->accept();
}

void ConvexHullWidget::wheelEvent(QWheelEvent* event) {
    if (!GetPlotRectangle().contains(event->position()) || event->angleDelta().y() == 0) {
        event->ignore();
        return;
    }

    const auto position = event->position();
    const auto worldBeforeZoom = ConvertToWorld(position);
    const auto delta = event->angleDelta().y();
    const auto zoomFactor = std::pow(ZoomStep, static_cast<qreal>(delta) / WheelDeltaPerStep);
    Zoom = std::clamp(Zoom * zoomFactor, MinimumZoom, MaximumZoom);

    const auto plot = GetPlotRectangle();
    const auto scaleX = plot.width() / (WorldBounds.width() / Zoom);
    const auto scaleY = plot.height() / (WorldBounds.height() / Zoom);
    ViewCenter = {
        worldBeforeZoom.x() - (position.x() - plot.center().x()) / scaleX,
        worldBeforeZoom.y() + (position.y() - plot.center().y()) / scaleY,
    };
    update();
    event->accept();
}

void ConvexHullWidget::updateHull() {
    Hull = Calculator.FindConvexHull(Points);
    update();
}

void DrawPolygon(const compg::ConvexHull2D& hull, QPainter& painter) {
    QPolygonF polygon;
    for (const auto& vertex : hull.Vertices) {
        polygon << QPointF{vertex[0], vertex[1]};
    }
    painter.setPen(Qt::NoPen);
    painter.setBrush(FillColor);
    painter.drawPolygon(polygon);
}

void DrawBorder(const compg::ConvexHull2D& hull, QPainter& painter) {
    QPen hullPen{HullColor, HullPenWidth};
    hullPen.setCosmetic(true);
    painter.setPen(hullPen);
    painter.setBrush(Qt::NoBrush);
    for (std::size_t index = 0; index < hull.Vertices.size(); ++index) {
        const auto& first = hull.Vertices.at(index);
        const auto& second = hull.Vertices.at((index + 1) % hull.Vertices.size());
        painter.drawLine(QLineF{first[0], first[1], second[0], second[1]});
    }
}

void ConvexHullWidget::paintEvent(QPaintEvent*) {
    QPainter painter{this};
    painter.setRenderHint(QPainter::Antialiasing);
    painter.fillRect(rect(), BackgroundColor);

    const auto plot = GetPlotRectangle();
    const auto scaleX = plot.width() / (WorldBounds.width() / Zoom);
    const auto scaleY = plot.height() / (WorldBounds.height() / Zoom);
    painter.save();
    painter.setClipRect(plot);
    painter.translate(plot.center());
    painter.scale(scaleX, -scaleY);
    painter.translate(-ViewCenter.x(), -ViewCenter.y());

    if (Hull.Vertices.size() >= 3) {
        DrawPolygon(Hull, painter);
    }

    if (Hull.Vertices.size() >= 2) {
        DrawBorder(Hull, painter);
    }

    painter.setPen(Qt::NoPen);
    painter.setBrush(PointColor);
    for (const auto& point : Points) {
        painter.drawEllipse(QPointF{point[0], point[1]}, PointRadius / scaleX, PointRadius / scaleY);
    }
    painter.restore();

    painter.setPen(QPen{BorderColor, BorderPenWidth});
    painter.setBrush(Qt::NoBrush);
    painter.drawRect(plot);
}