#pragma once

#include "thetastar/ThetaStar.h"

class AStar : public ThetaStar {
 public:
  AStar() : ThetaStar(true, "A*") {}
};
