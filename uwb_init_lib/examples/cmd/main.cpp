#include <iostream>
#include <string>
#include <fstream>
#include <map>
#include <vector>
#include <utility> // std::pair
#include <stdexcept> // std::runtime_error
#include <sstream> // std::stringstream

#include "include/CLI11.hpp"
#include "uwb_init.hpp"

using namespace uwb_init;

std::map<std::string, std::vector<double>> read_csv(std::string const& fn)

{
  // ref: https://www.gormanalysis.com/blog/reading-and-writing-csv-files-with-cpp/
  // Reads a CSV file into a vector of <string, vector<int>> pairs where
  // each pair represents <column name, column values>

          // Create a vector of <string, int vector> pairs to store the result
  std::vector<std::pair<std::string, std::vector<double>>> result;

          // Create an input filestream
  std::ifstream myFile(fn);

          // Make sure the file is open
  if(!myFile.is_open()) throw std::runtime_error("Could not open file");

          // Helper vars
  std::string line, colname;
  double val;

          // Read the column names
  if(myFile.good())
  {
    // Extract the first line in the file
    std::getline(myFile, line);

            // Create a stringstream from line
    std::stringstream ss(line);

            // Extract each column name
    while(std::getline(ss, colname, ',')){

      // Initialize and add <colname, int vector> pairs to result
      result.push_back({colname, std::vector<double> {}});
    }
  }

  // Read data, line by line
  while(std::getline(myFile, line))
  {
    // Create a stringstream of the current line
    std::stringstream ss(line);

    // Keep track of the current column index
    int colIdx = 0;

    // Extract each integer
    while(ss >> val){

      // Add the current integer to the 'colIdx' column's values vector
      result.at(colIdx).second.push_back(val);

      // If the next token is a comma, ignore it and move on
      if(ss.peek() == ',') ss.ignore();

      // Increment the column index
      colIdx++;
    }
  }

          // Close file
  myFile.close();


  std::map<std::string, std::vector<double>> m;
  for(auto const& e : result) {
    m.insert({e.first, e.second});
  }
  return m;
}

// Setup:
// p_BT_carr = {[1;0;0]; [-1;0;0]};
// bias_gamma_carr = {1.1; 1.5};
// p_GA = [10;10;10];
// sigma_d = 0.1; sigma_T = 0.1;
// --uwb_meas_csv /home/jungr/workspace/catkin_ws/test_uwb_init/uwb_init_lib/examples/cmd/data/DS_TWR_range.csv --tag_pos_csv /home/jungr/workspace/catkin_ws/test_uwb_init/uwb_init_lib/examples/cmd/data/DS_TWR_pos.csv
int main(int argc, char** argv)
{
  std::string app_name = "uwb_init_cmd";
  CLI::App app{app_name};


  std::string uwb_meas_csv = "";
  app.add_option("--uwb_meas_csv", uwb_meas_csv, "CSV file containing <t, ID_Tag, ID_Anchor, range>", true);
  std::string tag_pos_csv = "";
  app.add_option("--tag_pos_csv", tag_pos_csv, "CSV file containing <t, ID_Tag, x, y, z>", true);
  bool use_ransac = false;
  app.add_flag("--use_ransac", use_ransac, "select if ransac is turned on");
  bool use_double = false;
  app.add_flag("--use_double", use_double, "select if double or single method is used");
  double sigma_pos = 0.0;
  app.add_option("--sigma_pos", sigma_pos, "noise of position measurement");
  double sigma_range = 0.0;
  app.add_option("--sigma_range", sigma_range, "noise of distance measurement");

  CLI11_PARSE(app, argc, argv);
  // Print library info

  std::shared_ptr<UwbInitOptions> init_options = nullptr;
  std::unique_ptr<LsSolverOptions> ls_options = nullptr;
  std::unique_ptr<NlsSolverOptions> nls_options = nullptr;
  std::unique_ptr<PlannerOptions> planner_options = nullptr;

  RANSAC_Options ransac_options(0.99, 10, 0.15);

  if (use_double) {
    init_options = std::make_shared<UwbInitOptions>(InitMethod::DOUBLE, BiasType::CONST_BIAS, ransac_options);

  } else {
   init_options = std::make_shared<UwbInitOptions>(InitMethod::SINGLE, BiasType::CONST_BIAS, ransac_options);
  }
  ls_options = std::make_unique<LsSolverOptions>(sigma_pos, sigma_range, true, use_ransac);
  nls_options = std::make_unique<NlsSolverOptions>(1e-2, 10.0, 1e-6, 1e-6, 1e3);
  planner_options = std::make_unique<PlannerOptions>(10, 10, 3000, 0.5, 0.2, 2, 2, 4, 4, 5, 6, 1, 0, 0);

          // Test initialization
  UwbInitializer uwb_init(LoggerLevel::FULL, std::move(init_options), std::move(ls_options), std::move(nls_options),
                          std::move(planner_options));


  std::map<std::string, std::vector<double>> uwb_meas = read_csv(uwb_meas_csv);
  std::map<std::string, std::vector<double>> tag_pos = read_csv(tag_pos_csv);
  for(size_t i = 0; i< tag_pos.at("t").size(); i++)
  {
    double t = tag_pos.at("t")[i];
    Eigen::Vector3d pos(tag_pos.at("x")[i], tag_pos.at("y")[i], tag_pos.at("z")[i]);
    int Tag_ID = tag_pos.at("ID_Tag")[i];
    uwb_init.feed_position(t, pos, Tag_ID);
  }
  std::cout << "* Tag postions fed: " << tag_pos.at("t").size() << std::endl;

  for(size_t i = 0; i< uwb_meas.at("t").size(); i++) {

    double t = uwb_meas.at("t")[i];
    double r = uwb_meas.at("range")[i];
    int Tag_ID = uwb_meas.at("ID_Tag")[i];
    int Anchor_ID = uwb_meas.at("ID_Anchor")[i];
    uwb_init::UwbData uwb_data(true, r, Anchor_ID, Tag_ID);
    uwb_init.feed_uwb(t, uwb_data);
  }
  std::cout << "* Range measurements fed: " << uwb_meas.at("t").size() << std::endl;

  if (uwb_init.init_anchors())
  {
    LSSolutions ls_sols = uwb_init.get_ls_solutions();
    NLSSolutions nls_sols = uwb_init.get_nls_solutions();
  }

  if (uwb_init.compute_waypoints(Eigen::Vector3d::Zero()))
  {
    Waypoints opt_wps = uwb_init.get_waypoints();
  }

  if (uwb_init.refine_anchors())
  {
    NLSSolutions refined_sols = uwb_init.get_refined_solutions();
  }

  char c;
  std::cout << "press key + enter to continue..." << std::endl;
  std::cin >> c;
  return 0;
}
