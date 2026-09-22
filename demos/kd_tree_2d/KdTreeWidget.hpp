#pragma once

#include <QPointF>
#include <QWidget>

#include <memory>
#include <vector>

#include "common/Vertex.hpp"
#include "math/primitives/Box.hpp"
#include "range_query/kd_tree/KdTree.hpp"

class KdTreeWidget : public QWidget {
    Q_OBJECT

public:
    explicit KdTreeWidget(QWidget* parent = nullptr);

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
    using TreeType = compg::KdTree<2>;

    void RebuildTree();
    [[nodiscard]] QRectF GetPlotRectangle() const;
    [[nodiscard]] QPointF ConvertToWorld(const QPointF& widgetPosition) const;
    [[nodiscard]] QPointF ConvertToWidget(const compg::Vertex2D& worldPosition) const;
    [[nodiscard]] compg::Box2D GetVisibleWord() const;
    QPointF ViewCenter{0.0, 0.0};
    QPointF LastPanPosition;
    std::vector<compg::Vertex2D> Vertices;
    TreeType Tree{};
    double Zoom{1.0};
    bool IsPanning{false};
};
