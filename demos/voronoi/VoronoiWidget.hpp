#pragma once

#include <QPointF>
#include <QWidget>

#include <vector>

#include "common/Vertex.hpp"
#include "data_structures/DoublyConnectedEdgeList.hpp"
#include "voronoi/VoronoiCalculator.hpp"

class VoronoiWidget : public QWidget {
    Q_OBJECT

public:
    explicit VoronoiWidget(QWidget* parent = nullptr);

    void clear();

signals:
    void clicked(const QPointF& position);

protected:
    void mousePressEvent(QMouseEvent* event) override;
    void paintEvent(QPaintEvent* event) override;

private:
    void UpdateDiagram();
    QPointF ConvertToWorld(const QPointF& widgetPosition) const;
    QRectF GetPlotRectangle() const;

    std::vector<compg::Vertex2D> Sites;
    compg::VoronoiCalculator Calculator;
    compg::DoublyConnectedEdgeList Diagram;
    QPointF ViewCenter{0.0, 0.0};
};