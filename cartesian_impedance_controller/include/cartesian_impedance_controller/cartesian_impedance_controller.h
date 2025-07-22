#pragma once

#include "open_manipulator_p_controller/open_manipulator_p_controller.h"

namespace cartesian_impedance_controller
{
  class CartesianImpedanceController
  {
  public:
    CartesianImpedanceController();
    ~CartesianImpedanceController() = default;

    void setManipulator(open_manipulator::OpenManipulator* manipulator);  // <--- Add this


  private:

    open_manipulator::OpenManipulator* manipulator_;  // <--- Store the pointer


  };

} // namespace cartesian_impedance_controller
