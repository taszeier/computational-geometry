#pragma once

#include <QWidget>

#include <optional>
#include <vector>

#include "data_structures/DoublyConnectedEdgeList.hpp"
#include "quad_tree/CircuitSegment.hpp"

class CircuitQuadTreeWidget : public QWidget {
    Q_OBJECT

public:
    explicit CircuitQuadTreeWidget(QWidget* parent = nullptr);

    void clear();
    void SetPower(int power);

signals:
    void messageChanged(const QString& message);

protected:
    void mousePressEvent(QMouseEvent* event) override;
    void mouseMoveEvent(QMouseEvent* event) override;
    void mouseReleaseEvent(QMouseEvent* event) override;
    void paintEvent(QPaintEvent* event) override;
    void wheelEvent(QWheelEvent* event) override;

private:
    void DrawBoundarySquare(QPainter& painter) const;
    void DrawLattice(QPainter& painter) const;
    void DrawMesh(QPainter& painter) const;
    void DrawSegments(QPainter& painter) const;

private:
    using Segment = compg::CircuitSegment::segment_type;

    void RebuildTree();
    void TryToInsert(const Segment& segment);
    QPointF ConvertToWorld(const QPointF& widgetPosition) const;
    QPointF ConvertToWidget(const compg::CircuitVertex& worldPosition) const;
    compg::CircuitVertex SnapToGrid(const QPointF& position) const;
    QRectF GetPlotRectangle() const;
    double GetWorldSideLength() const;

    std::vector<compg::CircuitSegment> Segments;
    compg::DoublyConnectedEdgeList Mesh;
    int Power = 5;
    QPointF ViewCenter{16.0, 16.0};
    QPointF LastPointerPosition;
    QPointF DragStartPosition;
    std::optional<compg::CircuitVertex> CandidateStart;
    std::optional<compg::CircuitVertex> CandidateEnd;
    double Zoom{0.65};
    bool IsPanning = false;
    bool IsDrawing = false;
};