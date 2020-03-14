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

  struct vertex_3d {
    double x;
    double y;
    double z;
    double rot_x;
    double rot_y;
    double rot_z;
    double rot_w;
  };

  struct edge {
    int start;
    int end;
    double x;
    double y;
    double theta;
    vector<double> noise;
  };

  struct edge_3d {
    int start;
    int end;
    double x;
    double y;
    double z;
    double rot_x;
    double rot_y;
    double rot_z;
    double rot_w;
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

  int triangle_sum(int row, int col, int matSize) {
    int idx = 0;
    for (int i = 0; i < row; ++i) {
      idx += (matSize - i);
    }
    idx += (col - row);

    return idx;
  }

  noiseModel::Gaussian::shared_ptr buildNoise(vector<double> noiseVec, int matSize = 3) {

    string s = "";
    Matrix noise = Matrix(matSize, matSize);
    for (int i = 0; i < matSize; ++i) {
      for (int j = 0; j < matSize; ++j) {
        int temp_i = i;
        int temp_j = j;
        if (temp_i > temp_j) swap(temp_i, temp_j);
        noise(i, j) =  noiseVec[triangle_sum(temp_i, temp_j, matSize)];

      }
    }

    noiseModel::Gaussian::shared_ptr ret_noise = noiseModel::Gaussian::Information(noise);
    return ret_noise;

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

void print_values(Values values) {
  for(auto key_value = values.begin(); key_value != values.end(); ++key_value) {
    auto p = key_value->value.cast<Pose2>();
    cout << p.x() << " " << p.y() << endl;

  }
}
void one_b() {

  //pair<NonlinearFactorGraph::shared_ptr, Values::shared_ptr> res = dataloader(-1, file);
  NonlinearFactorGraph::shared_ptr graph;
  Values::shared_ptr initial;
  //string file = perturb("control.g2o");

  string file = perturb("control.g2o", "modified.txt");
  bool is3D = false;
  boost::tie(graph, initial) = readG2o(file, is3D);

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
  void print_values(Values values) {
    for(auto key_value = values.begin(); key_value != values.end(); ++key_value) {
      auto p = key_value->value.cast<Pose3>();
      cout << p.x() << " " << p.y() <<" " <<p.z() << endl;

    }
  }
  void two_b() {

      //pair<NonlinearFactorGraph::shared_ptr, Values::shared_ptr> res = dataloader(-1, file);
      NonlinearFactorGraph::shared_ptr graph;
      Values::shared_ptr initial;
      //string file = perturb("control.g2o");

      string file = perturb("3D_control.g2o", "modified_3D.txt");
      bool is3D = true;
      boost::tie(graph, initial) = readG2o("3D_control.g2o", is3D);

      GaussNewtonParams parameters;


      NonlinearFactorGraph graphWithPrior = *graph;
      noiseModel::Diagonal::shared_ptr priorModel = //
          noiseModel::Diagonal::Variances((Vector(6) << 0.09, 0.09, 0.09, 0.05, 0.05, 0.05).finished());

      graphWithPrior.add(PriorFactor<Pose3>(0, initial->begin()->value.cast<Pose3>(), priorModel));

      auto opt = GaussNewtonOptimizer (graphWithPrior, *initial, parameters).optimize();

      print_values(opt);
  }
  tuple<deque<vertex_3d>, deque<edge_3d>, unordered_map<string, edge_3d>> dataloader(string file) {
    ifstream is;
    ofstream vertex_output;
    ofstream between_output;
    ofstream close_loop_output;
    string temp = "";
    deque<vertex_3d> vertexList;
    deque<edge_3d> betweenEdgeList;
    unordered_map<string, edge_3d> closeLoopList;

    is.open(file);
    vertex_output.open("vertex_3D.txt");
    between_output.open("between_3D.txt");
    close_loop_output.open("close_loop_3D.txt");
    while(!is.eof()) {
      getline(is, temp);

      if (temp.find(" ") == string::npos) break;
      vector<string> v = split_string(temp);
      if (temp.find("VERTEX") != string::npos){

        vertex_output << temp + "\n";

        vertexList.push_back(vertex_3d{stod(v[2]), stod(v[3]), stod(v[4]), stod(v[5]), stod(v[6]), stod(v[7]), stod(v[8])});
      } else if (stoi(v[1]) + 1 == stoi(v[2])) {
        vector<double> noiseTemp;

        noiseTemp.resize(v.size() - 10);

        transform(v.begin() + 10, v.end(), noiseTemp.begin(), [](const string& val){return stod(val);});
        betweenEdgeList.push_back(edge_3d{stoi(v[1]), stoi(v[2]), stod(v[3]), stod(v[4]), stod(v[5]), stod(v[6]), stod(v[7]), stod(v[8]), stod(v[9]), noiseTemp});
        between_output << temp + "\n";
      } else {
        vector<double> noiseTemp;
        noiseTemp.resize(v.size() - 10);

        transform(v.begin() + 10, v.end(), noiseTemp.begin(), [](const string& val){return stod(val);});

        closeLoopList[v[1] + " " + v[2]] = edge_3d{stoi(v[1]), stoi(v[2]), stod(v[3]), stod(v[4]), stod(v[5]), stod(v[6]), stod(v[7]), stod(v[8]), stod(v[9]), noiseTemp};
        close_loop_output << temp + "\n";
      }

    }

    vertex_output.close();
    between_output.close();
    close_loop_output.close();
    return make_tuple(vertexList, betweenEdgeList, closeLoopList);
  }
  void two_c() {
      string file = "3D_control.g2o";
      ISAM2Params parameters;
      parameters.relinearizeThreshold = 0.01;
      parameters.relinearizeSkip = 1;
      parameters.cacheLinearizedFactors = false;
      parameters.enableDetailedResults = true;
      ISAM2 isam(parameters);
      tuple<deque<vertex_3d>, deque<edge_3d>, unordered_map<string, edge_3d>> vertexAndEdges = dataloader(file);
      deque<vertex_3d>& vertexList = get<0>(vertexAndEdges);
      deque<edge_3d>& betweenEdgeList = get<1>(vertexAndEdges);
      unordered_map<string, edge_3d>& closeLoopList = get<2>(vertexAndEdges);
      NonlinearFactorGraph graph;
      Values initial;
      noiseModel::Diagonal::shared_ptr priorModel = //
          noiseModel::Diagonal::Variances((Vector(6) << 0.09, 0.09, 0.09, 0.05, 0.05, 0.05).finished());
      graph.emplace_shared<PriorFactor<Pose3>>(0, Pose3(Rot3(vertexList[0].rot_w, vertexList[0].rot_x, vertexList[0].rot_y, vertexList[0].rot_z), Point3(vertexList[0].x, vertexList[0].y, vertexList[0].z)), priorModel);
      Values currentEstimate;

      for (size_t i = 0; i < vertexList.size(); ++ i) {
        initial.insert(i, Pose3(Rot3(vertexList[i].rot_w, vertexList[i].rot_x, vertexList[i].rot_y, vertexList[i].rot_z), Point3(vertexList[i].x, vertexList[i].y, vertexList[i].z)));

        if (i == 0) {

          isam.update(graph, initial);

          graph.resize(0);
          initial.clear();
          continue;
        }
        edge_3d betweenEdge = betweenEdgeList[i - 1];
        Pose3 betweenPose = Pose3(Rot3(betweenEdge.rot_w, betweenEdge.rot_x, betweenEdge.rot_y, betweenEdge.rot_z), Point3(betweenEdge.x, betweenEdge.y, betweenEdge.z));

        graph.emplace_shared<BetweenFactor<Pose3> >(betweenEdge.start, betweenEdge.end, betweenPose, buildNoise(betweenEdge.noise, 6));

        for (size_t j = 0; j < i; ++j) {

          if (closeLoopList.find(to_string(j) + " " + to_string(i)) != closeLoopList.end()) {
            edge_3d closeLoopEdge = closeLoopList[to_string(j) + " " + to_string(i)];
            Pose3 closeLoopPose = Pose3(Rot3(closeLoopEdge.rot_w, closeLoopEdge.rot_x, closeLoopEdge.rot_y, closeLoopEdge.rot_z), Point3(closeLoopEdge.x, closeLoopEdge.y, closeLoopEdge.z));

            graph.emplace_shared<BetweenFactor<Pose3> >(j, i, closeLoopPose, buildNoise(closeLoopEdge.noise, 6));
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
int main() {
  srand(time(NULL));
  Q1 q1 = Q1();
  //q1.one_b();
  //q1.one_c();

  Q2 q2 = Q2();
  //q2.two_b();
  q2.two_c();

  return 0;
}
