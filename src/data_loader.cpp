#include <gtsam/slam/dataset.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <iostream>
#include <fstream>
#include <string>
#include <optional>
#include <functional>
using namespace std;
using namespace gtsam;

bool dataloader(string file,  NonlinearFactorGraph::shared_ptr & graph, Values::shared_ptr& initial, int start, int end = -1) {
  ifstream is(file.c_str());
  if (end == -1) end = INT_MAX;

  string buffer_file_name = "temp.txt";
  ofstream buffer_file;
  buffer_file.open(buffer_file_name);
  int i = 0;
  string buffer;
  for (; i < start; ++i) {
    if (is.eof()) return false;
    getline(is, buffer);
  }
  for (; i < end && !is.eof(); ++i) {

    getline(is, buffer);
    buffer += "\n";
    buffer_file << buffer;
  }
  buffer_file.close();

  bool is3D = false;
  boost::tie(graph, initial) = readG2o(buffer_file_name);
  return true;

}

void print_values(Values values) {
  for(auto key_value = values.begin(); key_value != values.end(); ++key_value) {
    auto p = key_value->value.cast<Pose2>();
    cout << p.x() << " " << p.y() <<" " <<p.theta() << endl;
    cout << "\n";
  }
}

void one_b() {
  string file = "control.g2o";
  //pair<NonlinearFactorGraph::shared_ptr, Values::shared_ptr> res = dataloader(-1, file);
  NonlinearFactorGraph::shared_ptr graph;
  Values::shared_ptr initial;
  dataloader(file, graph, initial, 0);

  GaussNewtonParams parameters;
  // Stop iterating once the change in error between steps is less than this value
  parameters.relativeErrorTol = 1e-5;
  // Do not perform more than N iteration steps
  parameters.maxIterations = 100;
  NonlinearFactorGraph graphWithPrior = *graph;
  noiseModel::Diagonal::shared_ptr priorModel = //
      noiseModel::Diagonal::Variances(Vector3(1e-6, 1e-6, 1e-8));
  graphWithPrior.add(PriorFactor<Pose2>(0, Pose2(), priorModel));

  auto opt = GaussNewtonOptimizer (graphWithPrior, *initial, parameters).optimize();
  print_values(opt);

}

void one_c() {
  ISAM2Params parameters;
  parameters.relinearizeThreshold = 0.01;
  parameters.relinearizeSkip = 1;
  parameters.cacheLinearizedFactors = false;
  parameters.enableDetailedResults = true;
  ISAM2 isam(parameters);


  int pointer = 0;
  int flag = true;
  NonlinearFactorGraph::shared_ptr graph;
  Values::shared_ptr initial;
  string file = "control.g2o";
  while(dataloader(file,graph, initial, pointer, pointer + 50) == true) {

    //pair<NonlinearFactorGraph::shared_ptr, Values::shared_ptr> res = dataloader(-1, file);

    dataloader(file, graph, initial, pointer, pointer + 50);

    if (pointer == 0) {

      noiseModel::Diagonal::shared_ptr priorModel = //
          noiseModel::Diagonal::Variances(Vector3(1e-6, 1e-6, 1e-8));
      graph->add(PriorFactor<Pose2>(0, Pose2(), priorModel));
    }
    //isam.update(*graph, *initial);
    //isam.update();
    //Values result = isam.calculateEstimate();
    //result.print();
    cout << pointer << endl;
    //print_values(result);
    pointer += 50;

  }

}
int main() {
  one_b();
  //one_c();
  return 0;
}
