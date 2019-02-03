#include "gnode_base.h"

#include "base/PlannerSettings.h"

#if QT_SUPPORT
#include "gui/QtVisualizer.h"
#endif

bool GNode_base::isblock(double x, double y) {
  //    bool c = PlannerSettings::environment->collides(x, y);
  //    QtVisualizer::drawNode(x, y, c ? Qt::red : Qt::darkGreen, 0.05);
  //    return c;
  return PlannerSettings::environment->collides(x, y);
}

bool GNode_base::line(double x0, double y0, double y1, double x1) {
  double dx = (x1 - x0);
  double dy = (y1 - y0);
  const double size = std::sqrt(dx * dx + dy * dy);
  const double scale = 0.1;
  dx = dx / size * scale;
  dy = dy / size * scale;

  const auto steps = (int)(size / std::sqrt(dx * dx + dy * dy));

  for (int j = 1; j < steps; ++j) {
    if (GNode_base::isblock(x0 + dx * j, y0 + dy * j)) {
      return false;
    }
  }

  return true;
}

bool GNode_base::line(const GNode_base *successor,
                      const GNode_base *parent_node) {
  double x0, y0, y1, x1;
  x0 = parent_node->x;
  y0 = parent_node->y;
  x1 = successor->x;
  y1 = successor->y;
  bool c = line(x0, y0, y1, x1);
  //    QtVisualizer::drawPath({Tpoint(x0, y0), Tpoint(x1, y1)}, !c ? Qt::red :
  //    Qt::darkGreen);
  return c;
  //    return line(x0, y0, y1, x1);
}
