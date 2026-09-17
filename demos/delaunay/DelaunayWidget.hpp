#pragma once

#include <QWidget>

#include <vector>

#include "common/Vertex.hpp"
#include "data_structures/DoublyConnectedEdgeList.hpp"
#include "delaunay/DelaunayTriangulator.hpp"

class DelaunayWidget : public QWidget {
    Q_OBJECT

public:
    explicit DelaunayWidget(QWidget* parent = nullptr);
    void clear();

signals:
    void clicked(const QPointF& position);

protected:
    void mousePressEvent(QMouseEvent* event) override;
    void mouseMoveEvent(QMouseEvent* event) override;
    void mouseReleaseEvent(QMouseEvent* event) override;
    void paintEvent(QPaintEvent* event) override;
    void wheelEvent(QWheelEvent* event) override;

private:
    void UpdateTriangulation();
    QPointF ConvertToWorld(const QPointF& widgetPosition) const;
    QRectF GetPlotRectangle() const;

    static constexpr QPointF DefaultViewCenter{0.0, 0.0};
    static constexpr qreal DefaultZoom = 1.0;

    std::vector<compg::Vertex2D> Vertices;
    compg::DelaunayTriangulator Triangulator;
    compg::DoublyConnectedEdgeList Triangulation;

    QPointF ViewCenter{DefaultViewCenter};
    QPointF LastPanPosition;
    qreal Zoom{DefaultZoom};
    bool IsPanning{false};
};