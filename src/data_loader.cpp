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
  void print_values(Values values) {
    for(auto key_value = values.begin(); key_value != values.end(); ++key_value) {
      auto p = key_value->value.cast<Pose3>();
      cout << p.x() << " " << p.y() <<" " <<p.z() << endl;
    }
  }
  string perturb(string file, string modifiedFileName, string type = "VERTEX") {

    ofstream modified;
    ifstream is;
    string temp = "";

    is.open(file);
    modified.open(modifiedFileName);

    while(!is.eof()) {
      getline(is, temp);
      if (temp.find(" ") == string::npos) break;
      if (temp.find(type) != string::npos){

        vector<string> v = split_string(temp);
        for (size_t i = 2; i < v.size(); ++i) {

          v[i] = to_string(stod(v[i]) + ((double) rand() / (RAND_MAX) * 1)); // perturb the initial state
        }
        temp = "";
        for (auto i : v) {
          temp += i + " ";
        }
      }
      modified << temp + "\n";
    }

    return modifiedFileName;
  }
};
class Q1: public common_functions {

public:

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

  //pair<NonlinearFactorGraph::shared_ptr, Values::shared_ptr> res = dataloader(-1, file);
  NonlinearFactorGraph::shared_ptr graph;
  Values::shared_ptr initial;
  //string file = perturb("control.g2o");

  string file = perturb("control.g2o", "modified.txt");
  bool is3D = false;
  boost::tie(graph, initial) = readG2o(file, is3D);
  cout << "read" << endl;
  GaussNewtonParams parameters;
  // Stop iterating once the change in error between steps is less than this value
  parameters.relativeErrorTol = 1e-5;
  // Do not perform more than N iteration steps
  parameters.maxIterations = 100;
  NonlinearFactorGraph graphWithPrior = *graph;
  noiseModel::Diagonal::shared_ptr priorModel = //
      noiseModel::Diagonal::Variances(Vector3(0.1, 0.1, 0.3));
  graphWithPrior.add(PriorFactor<Pose2>(0, initial->begin()->value.cast<Pose2>(), priorModel));

  auto opt = GaussNewtonOptimizer (graphWithPrior, *initial, parameters).optimize();
  print_values(opt);
}

noiseModel::Gaussian::shared_ptr buildNoise(vector<double> noiseVec) {
  string s = "";
  Matrix noise = (Matrix(3,3) << noiseVec[0], noiseVec[1], noiseVec[2], noiseVec[1],noiseVec[3], noiseVec[4], noiseVec[2], noiseVec[4], noiseVec[5]).finished();

  noiseModel::Gaussian::shared_ptr ret_noise = noiseModel::Gaussian::Information(noise);
  return ret_noise;

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
    Values initial;
    graph.emplace_shared<PriorFactor<Pose2>>(0, Pose2(vertexList[0].x, vertexList[0].y, vertexList[0].theta), noiseModel::Diagonal::Variances(Vector3(1, 1, 1)));
    Values currentEstimate;
    for (size_t i = 0; i < vertexList.size(); ++ i) {
      initial.insert(i, Pose2(vertexList[i].x, vertexList[i].y, vertexList[i].theta));
      if (i == 0) {
        isam.update(graph, initial);
        graph.resize(0);
        initial.clear();
        continue;
      }
      edge betweenEdge = betweenEdgeList[i - 1];
      Pose2 betweenPose = Pose2(betweenEdge.x, betweenEdge.y, betweenEdge.theta);

      graph.emplace_shared<BetweenFactor<Pose2> >(betweenEdge.start, betweenEdge.end, betweenPose, buildNoise(betweenEdge.noise));

      for (size_t j = 0; j < i; ++j) {
        if (closeLoopList.find(to_string(j) + " " + to_string(i)) != closeLoopList.end()) {
          edge closeLoopEdge = closeLoopList[to_string(j) + " " + to_string(i)];
          Pose2 closeLoopPose = Pose2(closeLoopEdge.x, closeLoopEdge.y, closeLoopEdge.theta);
          graph.emplace_shared<BetweenFactor<Pose2> >(j, i, closeLoopPose, buildNoise(closeLoopEdge.noise));
        }
      }
      //print_values(initial);

      isam.update(graph, initial);
      isam.update();
      currentEstimate = isam.calculateEstimate();


      graph.resize(0);
      initial.clear();
    }
    print_values(currentEstimate);
  }
};
class Q2 : public common_functions {
public:
  void two_b() {

      //pair<NonlinearFactorGraph::shared_ptr, Values::shared_ptr> res = dataloader(-1, file);
      NonlinearFactorGraph::shared_ptr graph;
      Values::shared_ptr initial;
      //string file = perturb("control.g2o");

      string file = perturb("3D_control.g2o", "modified_3D.txt");
      bool is3D = true;
      boost::tie(graph, initial) = readG2o(file, is3D);
      cout << "read" << endl;
      GaussNewtonParams parameters;
      // Stop iterating once the change in error between steps is less than this value
      parameters.relativeErrorTol = 1e-3;
      // Do not perform more than N iteration steps
      parameters.maxIterations = 50;
      NonlinearFactorGraph graphWithPrior = *graph;
      noiseModel::Diagonal::shared_ptr priorModel = //
          noiseModel::Diagonal::Variances((Vector(6) << 0.1, 0.1, 0.1, 0.3, 0.3, 0.3).finished());
      cout << "noise" << endl;
      graphWithPrior.add(PriorFactor<Pose3>(0, initial->begin()->value.cast<Pose3>(), priorModel));
      cout << "prepped" << endl;
      //initial -> print();


      auto opt = GaussNewtonOptimizer (graphWithPrior, *initial, parameters).optimize();
      cout << "optimized" << endl;
      print_values(*initial);
  }
};
int main() {
  srand(time(NULL));
  Q1 q1 = Q1();
  //q1.one_b();
  //q1.one_c();

  Q2 q2 = Q2();
  q2.two_b();


  return 0;
}
