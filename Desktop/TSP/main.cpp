#include <iostream>
#include <vector>
#include <cmath>
#include <string>
#include <fstream>
#include <sstream>
#include <algorithm>
#include <limits>
#include <random>
#include <iomanip>
#include <chrono>
#include <cstdlib>

using namespace std;

struct City {
    int id;
    double x;
    double y;
};

// 经过优化的全局配置
struct TabuConfig {
    int max_iterations;    // 总迭代次数
    int tabu_tenure;       // 动态禁忌长度
    int max_idle;          // 允许的最大停滞次数（跳出局部最优的关键）

    TabuConfig(int n) {
        // 根据城市数量 n 动态调整参数
        max_iterations = 200000;             // 增加迭代次数
        tabu_tenure = (int)(n * 0.2);       // 禁忌长度取 20%
        if (tabu_tenure < 15) tabu_tenure = 15;
        max_idle = 5000;                    // 连续max_idle次没进步则触发重置
    }
};

class TSPSolver {
private:
    vector<City> cities;
    vector<vector<double>> distMatrix;
    int numCities;
    vector<vector<int>> tabuList;

public:
    TSPSolver() : numCities(0) {}

    double calcDistance(const City& c1, const City& c2) {
        double dx = c1.x - c2.x;
        double dy = c1.y - c2.y;
        return std::sqrt(dx * dx + dy * dy);
    }

    bool loadTSPFile(const string& filename) {
        ifstream file(filename);
        if (!file.is_open()) {
            cerr << "错误: 无法打开文件 " << filename << endl;
            return false;
        }

        string line;
        bool coordSection = false;
        cities.clear();
        while (getline(file, line)) {
            if (line.empty()) continue;
            if (line.find("NODE_COORD_SECTION") != string::npos) {
                coordSection = true;
                continue;
            }
            if (line.find("EOF") != string::npos) break;
            if (coordSection) {
                stringstream ss(line);
                int id; double x, y;
                if (ss >> id >> x >> y) cities.push_back({id, x, y});
            }
        }

        numCities = cities.size();
        if (numCities == 0) return false;

        distMatrix.assign(numCities, vector<double>(numCities));
        for (int i = 0; i < numCities; ++i) {
            for (int j = 0; j < numCities; ++j) {
                distMatrix[i][j] = calcDistance(cities[i], cities[j]);
            }
        }
        cout << "成功加载 " << numCities << " 个城市。" << endl;
        return true;
    }

    double getPathLength(const vector<int>& path) {
        double length = 0.0;
        for (int i = 0; i < numCities - 1; ++i) length += distMatrix[path[i]][path[i+1]];
        length += distMatrix[path[numCities-1]][path[0]];
        return length;
    }

    // 扰动函数：用于跳出局部最优。它通过随机交换路径中的节点并清空禁忌表，将搜索过程强行“推”到一个新的区域重新开始。
    void perturbSolution(vector<int>& path) {
        static mt19937 rng(time(0));
        for (int k = 0; k < 3; ++k) { // 随机交换3次
            int i = rng() % numCities;
            int j = rng() % numCities;
            swap(path[i], path[j]);
        }
    }

    void runTabuSearch(TabuConfig config) {
        if (numCities < 3) return;
        auto startTime = chrono::high_resolution_clock::now();

        vector<int> currentSol(numCities);
        for(int i=0; i<numCities; ++i) currentSol[i] = i;
        shuffle(currentSol.begin(), currentSol.end(), mt19937(random_device()()));

        double currentCost = getPathLength(currentSol);
        vector<int> bestSol = currentSol;
        double bestCost = currentCost;

        tabuList.assign(numCities, vector<int>(numCities, 0));
        int idleCounter = 0;

        cout << "初始路径长度: " << currentCost << endl;
        cout << "开始全局增强型禁忌搜索..." << endl;

        for (int iter = 0; iter < config.max_iterations; ++iter) {
            double bestNeighborCost = numeric_limits<double>::max();
            int best_i = -1, best_j = -1;

            // --- 全邻域遍历 2-Opt ---
            for (int i = 0; i < numCities - 1; ++i) {
                for (int j = i + 1; j < numCities; ++j) {
                    if (i == 0 && j == numCities - 1) continue;

                    // 计算增量 (Delta Evaluation)
                    int idx_p = (i - 1 + numCities) % numCities;
                    int idx_n = (j + 1) % numCities;
                    double delta = (distMatrix[currentSol[idx_p]][currentSol[j]] + distMatrix[currentSol[i]][currentSol[idx_n]]) -
                                   (distMatrix[currentSol[idx_p]][currentSol[i]] + distMatrix[currentSol[j]][currentSol[idx_n]]);

                    double neighborCost = currentCost + delta;

                    int u_t = min(currentSol[i], currentSol[j]);
                    int v_t = max(currentSol[i], currentSol[j]);

                    bool isTabu = (tabuList[u_t][v_t] > iter);
                    // 特赦准则
                    if (isTabu && neighborCost < bestCost) isTabu = false;

                    if (!isTabu && neighborCost < bestNeighborCost) {
                        bestNeighborCost = neighborCost;
                        best_i = i;
                        best_j = j;
                    }
                }
            }

            if (best_i != -1) {
                reverse(currentSol.begin() + best_i, currentSol.begin() + best_j + 1);
                currentCost = bestNeighborCost;

                if (currentCost < bestCost - 0.001) {
                    bestCost = currentCost;
                    bestSol = currentSol;
                    idleCounter = 0; // 重置停滞计数
                    cout << "迭代 " << iter << ": 发现新全局最优 = " << bestCost << endl;
                } else {
                    idleCounter++;
                }

                // 更新禁忌表
                tabuList[min(currentSol[best_i], currentSol[best_j])][max(currentSol[best_i], currentSol[best_j])] = iter + config.tabu_tenure;
            }

            // --- 跳出局部最优：重启策略 ---
            if (idleCounter > config.max_idle) {
                cout << "迭代 " << iter << ": 搜索停滞，执行扰动以跳出局部最优..." << endl;
                perturbSolution(currentSol);
                currentCost = getPathLength(currentSol);
                idleCounter = 0;
                // 清空部分禁忌表以增加灵活性
                tabuList.assign(numCities, vector<int>(numCities, 0));
            }
        }

        auto endTime = chrono::high_resolution_clock::now();
        cout << "\n---------------------------------" << endl;
        cout << "总时长: " << chrono::duration<double>(endTime - startTime).count() << " 秒" << endl;
        cout << "最终最优成本: " << bestCost << endl;
    }
};

int main(int argc, char* argv[]) {
    system("chcp 65001");
    string filename = (argc > 1) ? argv[1] : "a280.tsp";
    TSPSolver solver;
    if (solver.loadTSPFile(filename)) {
        // 使用针对全局最优改进的配置
        TabuConfig config(280);
        solver.runTabuSearch(config);
    }
    return 0;
}