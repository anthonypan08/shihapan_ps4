#include <gtsam/slam/dataset.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/geometry/Pose2.h>
#include <iostream>
#include <fstream>
#include <string>
using namespace std;
using namespace gtsam;

pair<NonlinearFactorGraph::shared_ptr, Values::shared_ptr> dataloader(int block, string file) {
  ifstream is(file.c_str());
  if (block == -1) block = INT_MAX;
  while(!is.eof()) {
    string buffer_file_name = "temp.txt";
    ofstream buffer_file;
    buffer_file.open(buffer_file_name);

    for (int i = 0; i < block && !is.eof(); ++i) {
      string buffer;
      getline(is, buffer);
      buffer += "\n";
      buffer_file << buffer;
    }
    buffer_file.close();
    NonlinearFactorGraph::shared_ptr graph;
    Values::shared_ptr initial;
    bool is3D = false;
    boost::tie(graph, initial) = readG2o(buffer_file_name);
    string write = "written.txt";
    //graph.get()->print();
    writeG2o(*(graph.get()), *(initial.get()), write);
    cout << "aaa" << endl;
    return make_pair(graph, initial);
  }

}
void one_b() {
  string file = "control.g2o";
  //pair<NonlinearFactorGraph::shared_ptr, Values::shared_ptr> res = dataloader(-1, file);
  NonlinearFactorGraph::shared_ptr graph;
  Values::shared_ptr initial;
  boost::tie(graph, initial) = readG2o(file);

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

  for(auto key_value = opt.begin(); key_value != opt.end(); ++key_value) {
    auto p = key_value->value.cast<Pose2>();

    cout << p.x() << " " << p.y() <<" " <<p.theta() << endl;
    cout << "\n";
  }
}
int main() {
  one_b();

  return 0;
}
