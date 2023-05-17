#include "planners/wps_gen.hpp"

using namespace uwb_init;

int main()
{
  Eigen::MatrixXd UWBs(4, 3);
  UWBs << -2.098, 3.903, 1.379, -0.817, -0.394, 0.152, 0.372, 1.713, 0.006, 1.175, -1.851, 0.023;
  Eigen::Vector3d p_k;
  p_k << 0, 0, 1;

  std::shared_ptr<Logger> logger = std::make_shared<Logger>(LoggerLevel::FULL);
  std::unique_ptr<PlannerOptions> options =
      std::make_unique<PlannerOptions>(10, 10, 3000, 0.5, 0.2, 2, 2, 4, 4, 5, 6, 1);

  OptWpsGenerator wps_gen(logger, std::move(options));
  Eigen::MatrixXd final_p_optimal = wps_gen.generate_wps(UWBs, p_k);
  std::cout << final_p_optimal << std::endl;
}
