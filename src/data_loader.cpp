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
#include <vector>
using namespace std;
using namespace gtsam;
class common_functions {
public:
  vector<string> split_string(string str) {
   vector<string> v;

   char cstring[str.size() + 1];
   strcpy(cstring, str.c_str());
   char* pch = strtok (cstring," ");
   while (pch != NULL)
   {
     v.push_back(pch);
     pch = strtok (NULL, " ");
   }
   return v;
  }
};
class Q1: public common_functions {
  struct vertex {
    double x;
    double y;
    double theta;
  };

  struct edge {
    int start;
    int end;
    double x;
    double y;
    double theta;
    vector<double> noise;
  };
public:
  void print_values(Values values) {
    for(auto key_value = values.begin(); key_value != values.end(); ++key_value) {
      auto p = key_value->value.cast<Pose2>();
      cout << p.x() << " " << p.y() <<" " <<p.theta() << endl;
      cout << "\n";
    }
  }
tuple<deque<vertex>, deque<edge>, unordered_map<string, edge>> dataloader(string file) {
  ifstream is;
  ofstream vertex_output;
  ofstream between_output;
  ofstream close_loop_output;
  string temp = "";
  deque<vertex> vertexList;
  deque<edge> betweenEdgeList;
  unordered_map<string, edge> closeLoopList;

  is.open(file);
  vertex_output.open("vertex.txt");
  between_output.open("between.txt");
  close_loop_output.open("close_loop.txt");
  while(!is.eof()) {
    getline(is, temp);

    if (temp.find(" ") == string::npos) break;
    vector<string> v = split_string(temp);
    if (temp.find("VERTEX") != string::npos){
      vertex_output << temp + "\n";
      vertexList.push_back(vertex{stod(v[2]), stod(v[3]), stod(v[4])});
    } else if (stoi(v[1]) + 1 == stoi(v[2])) {
      vector<double> noiseTemp = vector<double>{stod(v[6]), stod(v[7]), stod(v[8]), stod(v[9]), stod(v[10]), stod(v[11])};
      betweenEdgeList.push_back(edge{stoi(v[1]), stoi(v[2]), stod(v[3]), stod(v[4]), stod(v[5]), noiseTemp});
      between_output << temp + "\n";
    } else {
      vector<double> noiseTemp = vector<double>{stod(v[6]), stod(v[7]), stod(v[8]), stod(v[9]), stod(v[10]), stod(v[11])};
      closeLoopList[v[1] + " " + v[2]] = edge{stoi(v[1]), stoi(v[2]), stod(v[3]), stod(v[4]), stod(v[5]), noiseTemp};
      close_loop_output << temp + "\n";
    }

  }
  vertex_output.close();
  between_output.close();
  close_loop_output.close();
  return make_tuple(vertexList, betweenEdgeList, closeLoopList);
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
      noiseModel::Diagonal::Variances(Vector3(1, 1, 1));



  graphWithPrior.add(PriorFactor<Pose2>(0, initial->begin()->value.cast<Pose2>(), priorModel));
  for(size_t i = 0; i < initial->size(); ++i) {
    auto cur = initial->at<Pose2>(i);
    Pose2 dummy = Pose2(cur.x() * 1.0, cur.y() * 1.0, cur.theta() * 1.0);
    initial -> erase(i);
    initial -> insert(i, dummy);

  }
  //initial -> print();
  auto opt = GaussNewtonOptimizer (graphWithPrior, *initial, parameters).optimize();
  //print_values(opt);
  auto p = initial->begin()->value.cast<Pose2>();
  cout << p.x() << " " << p.y() <<" " <<p.theta() << endl;

}

void one_c() {
  string file = "control.g2o";
  ISAM2Params parameters;
  parameters.relinearizeThreshold = 0.01;
  parameters.relinearizeSkip = 1;
  parameters.cacheLinearizedFactors = false;
  parameters.enableDetailedResults = true;
  ISAM2 isam(parameters);

  tuple<deque<vertex>, deque<edge>, unordered_map<string, edge>> vertexAndEdges = dataloader(file);
  deque<vertex>& vertexList = get<0>(vertexAndEdges);
  deque<edge>& betweenEdgeList = get<1>(vertexAndEdges);
  unordered_map<string, edge>& closeLoopList = get<2>(vertexAndEdges);

  NonlinearFactorGraph graph;
  Values::shared_ptr initial;
  noiseModel::Diagonal::shared_ptr priorNoise = noiseModel::Diagonal::Sigmas(Vector3(0.3, 0.3, 0.1));
  graph.emplace_shared<PriorFactor<Pose2>>(0, Pose2(vertexList.x, vertexList.y, vertexList.theta), priorNoise);
  cout << "abc" << endl;

  for (int i = 1; i < vertexList; ++ i) {
    cout << i.x << endl;
  }




};
};
int main() {
  Q1 q1 = Q1();
  //q1.one_b();
  q1.one_c();

  return 0;
}
