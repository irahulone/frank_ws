#include <cartesian_impedance_controller/cartesian_impedance_controller.h>
#include "pseudo_inversion.h"


namespace cartesian_impedance_controller
{
  void cartesian_impedance_controller::CartesianImpedanceController::setManipulator(open_manipulator::OpenManipulator* manipulator) 
  {
    manipulator_ = manipulator;
  }


}

