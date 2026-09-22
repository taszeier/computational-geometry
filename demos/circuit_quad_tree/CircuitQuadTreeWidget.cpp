#include "CircuitQuadTreeWidget.hpp"

#include <QMouseEvent>
#include <QPainter>
#include <QWheelEvent>

#include <algorithm>
#include <cmath>
#include <ranges>

#include "CircuitSegmentValidator.hpp"
#include "data_structures/DoublyConnectedEdgeListAlgorithms.hpp"
#include "quad_tree/CircuitQuadTree.hpp"

#include <iostream>

namespace {
    constexpr double PlotMargin = 32.0;
}

CircuitQuadTreeWidget::CircuitQuadTreeWidget(QWidget* parent)
    : QWidget{parent} {
    setMinimumSize(520, 520);
    setMouseTracking(true);
    setAutoFillBackground(false);
}

void CircuitQuadTreeWidget::clear() {
    Segments.clear();
    Mesh = {};
    CandidateStart.reset();
    CandidateEnd.reset();
    update();
    emit messageChanged("Click and drag to add an integer circuit segment");
}

void CircuitQuadTreeWidget::SetPower(int power) {
    if (Power == power) {
        return;
    }
    Power = power;
    ViewCenter = {GetWorldSideLength() * 0.5, GetWorldSideLength() * 0.5};
    clear();
}

double CircuitQuadTreeWidget::GetWorldSideLength() const {
    return static_cast<double>(1ULL << Power);
}

QRectF CircuitQuadTreeWidget::GetPlotRectangle() const {
    return {PlotMargin, PlotMargin, width() - 2.0 * PlotMargin, height() - 2.0 * PlotMargin};
}

QPointF CircuitQuadTreeWidget::ConvertToWorld(const QPointF& widgetPosition) const {
    const auto plot = GetPlotRectangle();
    const auto scale = plot.width() / GetWorldSideLength() * Zoom;
    const auto x = ViewCenter.x() + (widgetPosition.x() - plot.center().x()) / scale;
    const auto y = ViewCenter.y() - (widgetPosition.y() - plot.center().y()) / scale;
    return {x, y};
}

QPointF CircuitQuadTreeWidget::ConvertToWidget(const compg::CircuitVertex& worldPosition) const {
    const auto plot = GetPlotRectangle();
    const auto scale = plot.width() / GetWorldSideLength() * Zoom;
    const auto x = plot.center().x() + (static_cast<double>(worldPosition[0]) - ViewCenter.x()) * scale;
    const auto y = plot.center().y() - (static_cast<double>(worldPosition[1]) - ViewCenter.y()) * scale;
    return {x, y};
}

compg::CircuitVertex CircuitQuadTreeWidget::SnapToGrid(const QPointF& position) const {
    const auto side = static_cast<unsigned int>(GetWorldSideLength());
    const auto x = std::clamp(std::llround(position.x()), 0LL, static_cast<long long>(side));
    const auto y = std::clamp(std::llround(position.y()), 0LL, static_cast<long long>(side));
    return {static_cast<unsigned int>(x), static_cast<unsigned int>(y)};
}

void CircuitQuadTreeWidget::mousePressEvent(QMouseEvent* event) {
    if (!GetPlotRectangle().contains(event->position())) {
        event->ignore();
        return;
    }

    if (event->button() == Qt::LeftButton) {
        CandidateStart = SnapToGrid(ConvertToWorld(event->position()));
        CandidateEnd = CandidateStart;
        IsDrawing = true;
        update();
        event->accept();
        return;
    }

    if (event->button() == Qt::RightButton) {
        IsPanning = true;
        LastPointerPosition = event->position();
        setCursor(Qt::ClosedHandCursor);
        event->accept();
        return;
    }
    event->ignore();
}

void CircuitQuadTreeWidget::mouseMoveEvent(QMouseEvent* event) {
    if (IsDrawing) {
        CandidateEnd = SnapToGrid(ConvertToWorld(event->position()));
        update();
        event->accept();
        return;
    }
    if (IsPanning) {
        const auto delta = event->position() - LastPointerPosition;
        const auto scale = GetPlotRectangle().width() / GetWorldSideLength() * Zoom;
        ViewCenter.rx() -= delta.x() / scale;
        ViewCenter.ry() += delta.y() / scale;
        LastPointerPosition = event->position();
        update();
        event->accept();
        return;
    }
    event->ignore();
}

void CircuitQuadTreeWidget::mouseReleaseEvent(QMouseEvent* event) {
    if (event->button() == Qt::LeftButton && IsDrawing) {
        IsDrawing = false;
        CandidateEnd = SnapToGrid(ConvertToWorld(event->position()));
        if (CandidateStart.has_value() && CandidateEnd.has_value()) {
            TryToInsert(Segment{CandidateStart.value(), CandidateEnd.value()});
        }
        CandidateStart.reset();
        CandidateEnd.reset();
        update();
        event->accept();
        return;
    }
    if (event->button() == Qt::RightButton && IsPanning) {
        IsPanning = false;
        unsetCursor();
        event->accept();
        return;
    }
    event->ignore();
}

void CircuitQuadTreeWidget::wheelEvent(QWheelEvent* event) {
    if (!GetPlotRectangle().contains(event->position()) || event->angleDelta().y() == 0) {
        event->ignore();
        return;
    }
    const auto cursorWorldPosition = ConvertToWorld(event->position());
    const auto zoomFactor = event->angleDelta().y() > 0 ? 1.15 : 1.0 / 1.15;
    Zoom = std::clamp(Zoom * zoomFactor, 0.25, 8.0);
    const auto newCursorWorldPosition = ConvertToWorld(event->position());
    ViewCenter += cursorWorldPosition - newCursorWorldPosition;
    update();
    event->accept();
}

void CircuitQuadTreeWidget::TryToInsert(const Segment& segment) {
    const auto result = ValidateCircuitSegment(segment, Segments, static_cast<std::size_t>(Power));
    if (!result.IsValid()) {
        emit messageChanged(QString::fromStdString(result.GetMessage()));
        return;
    }
    Segments.emplace_back(segment);
    RebuildTree();
    emit messageChanged("Segment inserted");
}

void CircuitQuadTreeWidget::RebuildTree() {
    Mesh = compg::CircuitQuadTree{Segments, static_cast<std::size_t>(Power)}.CreateMesh();
    update();
}

void CircuitQuadTreeWidget::DrawBoundarySquare(QPainter& painter) const {
    QPen boxPen{QColor{71, 85, 105}, 1.2};
    boxPen.setCosmetic(true);
    painter.setPen(boxPen);
    painter.setBrush(Qt::NoBrush);
    const auto side = static_cast<unsigned int>(GetWorldSideLength());
    painter.drawRect(QRectF{ConvertToWidget({0, side}), ConvertToWidget({side, 0})});
}

void CircuitQuadTreeWidget::DrawLattice(QPainter& painter) const {
    const auto scale = GetPlotRectangle().width() / GetWorldSideLength() * Zoom;
    const auto side = static_cast<unsigned int>(GetWorldSideLength());
    const auto latticeStep = static_cast<unsigned int>(std::max(1.0, std::ceil(3.0 / scale)));
    if (latticeStep == 1) {
        painter.setPen(Qt::NoPen);
        painter.setBrush(QColor{100, 116, 139, 110});
        for (unsigned int x = 0; x <= side; ++x) {
            for (unsigned int y = 0; y <= side; ++y) {
                painter.drawEllipse(ConvertToWidget({x, y}), 1.2, 1.2);
            }
        }
    } else {
        QPen latticePen{QColor{148, 163, 184, 70}, 0.6};
        latticePen.setCosmetic(true);
        painter.setPen(latticePen);
        for (unsigned int coordinate = 0; coordinate <= side; coordinate += latticeStep) {
            painter.drawLine(QLineF{ConvertToWidget({coordinate, 0}), ConvertToWidget({coordinate, side})});
            painter.drawLine(QLineF{ConvertToWidget({0, coordinate}), ConvertToWidget({side, coordinate})});
        }
    }
}

void CircuitQuadTreeWidget::DrawMesh(QPainter& painter) const {
    QPen meshPen{QColor{148, 163, 184}, 0.8};
    meshPen.setCosmetic(true);
    painter.setPen(meshPen);
    compg::WalkUndirectedEdges(Mesh, [this, &painter](auto edgeIndex) {
        const auto edge = Mesh.GetEdgeAsLineSegment(edgeIndex);
        painter.drawLine(QLineF{ConvertToWidget({edge[0][0], edge[0][1]}), ConvertToWidget({edge[1][0], edge[1][1]})});
    });
}

void CircuitQuadTreeWidget::DrawSegments(QPainter& painter) const {
    QPen segmentPen{QColor{14, 116, 144}, 2.4};
    segmentPen.setCosmetic(true);
    painter.setPen(segmentPen);
    for (const auto& segment : Segments) {
        painter.drawLine(QLineF{ConvertToWidget(segment[0]), ConvertToWidget(segment[1])});
    }

    if (CandidateStart.has_value() && CandidateEnd.has_value()) {
        QPen candidatePen{QColor{220, 38, 38}, 2.0, Qt::DashLine};
        candidatePen.setCosmetic(true);
        painter.setPen(candidatePen);
        painter.drawLine(QLineF{ConvertToWidget(CandidateStart.value()), ConvertToWidget(CandidateEnd.value())});
    }
}

void CircuitQuadTreeWidget::paintEvent(QPaintEvent*) {
    QPainter painter{this};
    painter.setRenderHint(QPainter::Antialiasing);
    painter.fillRect(rect(), QColor{248, 250, 252});

    const auto plot = GetPlotRectangle();
    painter.save();
    painter.setClipRect(plot);

    DrawBoundarySquare(painter);
    DrawLattice(painter);
    DrawMesh(painter);
    DrawSegments(painter);

    painter.restore();

    painter.setPen(QPen{QColor{100, 116, 139}, 1.0});
    painter.setBrush(Qt::NoBrush);
    painter.drawRect(plot);

    painter.setPen(QColor{71, 85, 105});
    painter.drawText(plot.adjusted(6, 6, -6, -6), Qt::AlignTop | Qt::AlignRight, QString{"power = %1"}.arg(Power));
}