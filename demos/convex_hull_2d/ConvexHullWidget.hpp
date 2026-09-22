#pragma once

#include <QWidget>

#include <vector>

#include "common/Vertex.hpp"
#include "convex_hull/AndrewsMonotoneChain.hpp"
#include "convex_hull/ConvexHull.hpp"

class ConvexHullWidget : public QWidget {
    Q_OBJECT

public:
    explicit ConvexHullWidget(QWidget* parent = nullptr);

    void clear();

signals:
    void clicked(const QPointF& position);

protected:
    void mousePressEvent(QMouseEvent* event) override;
    void mouseMoveEvent(QMouseEvent* event) override;
    void mouseReleaseEvent(QMouseEvent* event) override;
    void wheelEvent(QWheelEvent* event) override;
    void paintEvent(QPaintEvent* event) override;

private:
    void updateHull();
    QPointF ConvertToWorld(const QPointF& widgetPosition) const;
    QRectF GetPlotRectangle() const;

    std::vector<compg::Vertex2D> Points;
    compg::AndrewsMonotoneChain Calculator;
    compg::ConvexHull2D Hull;
    QPointF ViewCenter{0.0, 0.0};
    QPointF LastPanPosition;
    double Zoom{1.0};
    bool IsPanning{false};
};