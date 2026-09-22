#include "KdTreeWidget.hpp"

#include "math/Conversions.hpp"

#include <QPainter>
#include <QWheelEvent>

#include <algorithm>
#include <cmath>
#include <variant>

namespace {
    constexpr double PlotMargin = 32.0;
    constexpr double InitialWorldWidth = 100.0;
} // namespace

using namespace compg;

KdTreeWidget::KdTreeWidget(QWidget* parent)
    : QWidget{parent} {
    setMinimumSize(520, 420);
    setMouseTracking(true);
    setAutoFillBackground(false);
}

void KdTreeWidget::clear() {
    Vertices.clear();
    Tree = {};
    update();
}

QRectF KdTreeWidget::GetPlotRectangle() const {
    return {PlotMargin, PlotMargin, width() - 2.0 * PlotMargin, height() - 2.0 * PlotMargin};
}

QPointF KdTreeWidget::ConvertToWorld(const QPointF& widgetPosition) const {
    const auto plot = GetPlotRectangle();
    const auto scaleX = plot.width() / InitialWorldWidth * Zoom;
    const auto scaleY = plot.height() / InitialWorldWidth * Zoom;
    const auto x = ViewCenter.x() + (widgetPosition.x() - plot.center().x()) / scaleX;
    const auto y = ViewCenter.y() - (widgetPosition.y() - plot.center().y()) / scaleY;
    return {x, y};
}

QPointF KdTreeWidget::ConvertToWidget(const compg::Vertex2D& worldPosition) const {
    const auto plot = GetPlotRectangle();
    const auto scaleX = plot.width() / InitialWorldWidth * Zoom;
    const auto scaleY = plot.height() / InitialWorldWidth * Zoom;
    const auto x = plot.center().x() + (worldPosition[0] - ViewCenter.x()) * scaleX;
    const auto y = plot.center().y() - (worldPosition[1] - ViewCenter.y()) * scaleY;
    return {x, y};
}

void KdTreeWidget::mousePressEvent(QMouseEvent* event) {
    if (!GetPlotRectangle().contains(event->position())) {
        event->ignore();
        return;
    }

    if (event->button() == Qt::RightButton) {
        IsPanning = true;
        LastPanPosition = event->position();
        setCursor(Qt::ClosedHandCursor);
        event->accept();
        return;
    }

    if (event->button() == Qt::LeftButton) {
        const auto position = ConvertToWorld(event->position());
        const compg::Vertex2D vertex{position.x(), position.y()};
        if (std::ranges::find(Vertices, vertex) == Vertices.end()) {
            Vertices.push_back(vertex);
            RebuildTree();
        }
        emit clicked(position);
        event->accept();
        return;
    }
    event->ignore();
}

void KdTreeWidget::mouseMoveEvent(QMouseEvent* event) {
    if (!IsPanning) {
        event->ignore();
        return;
    }

    const auto delta = event->position() - LastPanPosition;
    const auto plot = GetPlotRectangle();
    ViewCenter.rx() -= delta.x() / (plot.width() / InitialWorldWidth * Zoom);
    ViewCenter.ry() += delta.y() / (plot.height() / InitialWorldWidth * Zoom);
    LastPanPosition = event->position();
    update();
    event->accept();
}

void KdTreeWidget::mouseReleaseEvent(QMouseEvent* event) {
    if (event->button() != Qt::RightButton || !IsPanning) {
        event->ignore();
        return;
    }

    IsPanning = false;
    unsetCursor();
    event->accept();
}

void KdTreeWidget::wheelEvent(QWheelEvent* event) {
    if (!GetPlotRectangle().contains(event->position()) || event->angleDelta().y() == 0) {
        event->ignore();
        return;
    }

    const auto cursorWorldPosition = ConvertToWorld(event->position());
    const auto zoomFactor = std::pow(1.15, static_cast<double>(event->angleDelta().y()) / 120.0);
    Zoom = std::clamp(Zoom * zoomFactor, 0.2, 8.0);
    const auto newCursorWorldPosition = ConvertToWorld(event->position());
    ViewCenter += cursorWorldPosition - newCursorWorldPosition;
    update();
    event->accept();
}

void KdTreeWidget::RebuildTree() {
    Tree = TreeType{Vertices};
    update();
}

QLineF Crop(const AxesAlignedHyperplane<2>& plane, const UnboundedBox<2>& region) {
    if (plane.GetAxisIndex() == 0) {
        return QLineF{
            plane.GetIntersection(), (*region.GetLowerCorner())[1], plane.GetIntersection(),
            (*region.GetUpperCorner())[1]
        };
    }
    return QLineF{
        (*region.GetLowerCorner())[0], plane.GetIntersection(), (*region.GetUpperCorner())[0], plane.GetIntersection()
    };
}

void DrawPlanes(const auto& node, const UnboundedBox<2>& region, std::size_t depth, QPainter& painter) {
    if (!node->IsLeaf()) {
        const auto& internal = std::get<KdTreeInternalNode<2>>(node->State);
        QPen planePen{
            QColor::fromHsvF(std::fmod(0.58 + 0.30 * static_cast<double>(depth), 1.0), 0.88, 0.88, 0.86), 1.0
        };
        planePen.setCosmetic(true);
        painter.setPen(planePen);
        const auto leftRegion = region.Intersected(internal.Plane, Side::Negative);
        const auto rightRegion = region.Intersected(internal.Plane, Side::Positive);
        painter.drawLine(Crop(internal.Plane, region));
        DrawPlanes(internal.LeftChild, leftRegion, depth + 1, painter);
        DrawPlanes(internal.RightChild, rightRegion, depth + 1, painter);
    }
}

void DrawPlanes(const KdTree<2>& tree, const Box2D& visibleRegion, QPainter& painter) {
    const auto& root = tree.GetRoot();
    if (root != nullptr) {
        const auto visibleRegionBox = ConvertTo<UnboundedBox>(visibleRegion);
        DrawPlanes(root, visibleRegionBox, 0, painter);
    }
}

Box2D KdTreeWidget::GetVisibleWord() const {
    const auto plot = GetPlotRectangle();
    const auto worldBottomLeft = ConvertToWorld(plot.bottomLeft());
    const auto worldTopRight = ConvertToWorld(plot.topRight());
    return {{worldBottomLeft.x(), worldBottomLeft.y()}, {worldTopRight.x(), worldTopRight.y()}};
}

void KdTreeWidget::paintEvent(QPaintEvent*) {
    QPainter painter{this};
    painter.setRenderHint(QPainter::Antialiasing);
    painter.fillRect(rect(), QColor{248, 250, 252});

    const auto plot = GetPlotRectangle();
    const auto scaleX = plot.width() / InitialWorldWidth * Zoom;
    const auto scaleY = plot.height() / InitialWorldWidth * Zoom;

    painter.save();
    painter.setClipRect(plot);
    painter.translate(plot.center());
    painter.scale(scaleX, -scaleY);
    painter.translate(-ViewCenter.x(), -ViewCenter.y());

    DrawPlanes(Tree, GetVisibleWord(), painter);

    painter.setPen(Qt::NoPen);
    painter.setBrush(QColor{220, 38, 38});
    for (const auto& vertex : Vertices) {
        painter.drawEllipse(QPointF{vertex[0], vertex[1]}, 3.5 / scaleX, 3.5 / scaleY);
    }
    painter.restore();

    painter.setPen(QPen{QColor{100, 116, 139}, 1.0});
    painter.setBrush(Qt::NoBrush);
    painter.drawRect(plot);
}
