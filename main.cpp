#include <iostream>
#include <vector>
#include <cmath>
#include <algorithm>
#include <random>
#include <ctime>
#include <climits>
#include <iomanip>
#include <queue>

using namespace std;

// 禁忌表项结构
struct TabuMove {
    int i;      // 城市i
    int j;      // 城市j
    int tenure; // 禁忌长度（剩余禁忌期）
    
    TabuMove(int _i, int _j, int _tenure) : i(_i), j(_j), tenure(_tenure) {}
};

// 旅行商问题类
class TSP {
private:
    int n; // 城市数量
    vector<pair<double, double>> cities; // 城市坐标
    vector<vector<double>> distance;    // 距离矩阵
    
    // 禁忌表
    vector<vector<int>> tabuTable;
    
    // 禁忌搜索参数
    int tabuTenure;      // 禁忌长度
    int maxIterations;   // 最大迭代次数
    int maxNoImprove;    // 最大无改进迭代次数
    
public:
    TSP(const vector<pair<double, double>>& _cities) 
        : cities(_cities), n(_cities.size()) {
        
        // 初始化距离矩阵
        distance.resize(n, vector<double>(n, 0));
        for (int i = 0; i < n; i++) {
            for (int j = i + 1; j < n; j++) {
                double dx = cities[i].first - cities[j].first;
                double dy = cities[i].second - cities[j].second;
                distance[i][j] = distance[j][i] = sqrt(dx * dx + dy * dy);
            }
        }
        
        // 初始化禁忌表
        tabuTable.resize(n, vector<int>(n, 0));
        
        // 设置默认参数
        tabuTenure = 10 + n / 5;    // 禁忌长度与问题规模相关
        maxIterations = 1000 * n;
        maxNoImprove = 100 * n;
    }
    
    // 计算路径总长度
    double calculateTotalDistance(const vector<int>& path) {
        double total = 0;
        for (int i = 0; i < n - 1; i++) {
            total += distance[path[i]][path[i + 1]];
        }
        total += distance[path[n - 1]][path[0]]; // 回到起点
        return total;
    }
    
    // 生成初始解（使用最近邻法）
    vector<int> generateInitialSolution() {
        vector<int> path(n);
        vector<bool> visited(n, false);
        
        // 随机选择起点
        random_device rd;
        mt19937 gen(rd());
        uniform_int_distribution<> dis(0, n - 1);
        int start = dis(gen);
        
        path[0] = start;
        visited[start] = true;
        
        // 最近邻法构建路径
        for (int i = 1; i < n; i++) {
            int current = path[i - 1];
            int nextCity = -1;
            double minDist = INFINITY;
            
            for (int j = 0; j < n; j++) {
                if (!visited[j] && distance[current][j] < minDist) {
                    minDist = distance[current][j];
                    nextCity = j;
                }
            }
            
            path[i] = nextCity;
            visited[nextCity] = true;
        }
        
        return path;
    }
    
    // 2-opt邻域操作
    double perform2OptMove(vector<int>& path, int i, int j) {
        // 计算当前边的长度
        int n = path.size();
        int i_prev = (i - 1 + n) % n;
        int j_next = (j + 1) % n;
        
        double oldDistance = distance[path[i_prev]][path[i]] + distance[path[j]][path[j_next]];
        double newDistance = distance[path[i_prev]][path[j]] + distance[path[i]][path[j_next]];
        
        // 如果新距离更短，执行交换
        if (newDistance < oldDistance) {
            // 反转i到j之间的路径段
            while (i < j) {
                swap(path[i], path[j]);
                i++;
                j--;
            }
            return newDistance - oldDistance;
        }
        
        return 0; // 没有改进
    }
    
    // 交换两个城市的位置
    double performSwapMove(vector<int>& path, int i, int j) {
        if (i == j) return 0;
        
        int n = path.size();
        int i_prev = (i - 1 + n) % n;
        int i_next = (i + 1) % n;
        int j_prev = (j - 1 + n) % n;
        int j_next = (j + 1) % n;
        
        // 处理相邻城市的情况
        if ((i + 1) % n == j) {
            // i和j相邻，且j在i后面
            double oldDist = distance[path[i_prev]][path[i]] + 
                           distance[path[i]][path[j]] + 
                           distance[path[j]][path[j_next]];
            double newDist = distance[path[i_prev]][path[j]] + 
                           distance[path[j]][path[i]] + 
                           distance[path[i]][path[j_next]];
            swap(path[i], path[j]);
            return newDist - oldDist;
        }
        else if ((j + 1) % n == i) {
            // j和i相邻，且i在j后面
            double oldDist = distance[path[j_prev]][path[j]] + 
                           distance[path[j]][path[i]] + 
                           distance[path[i]][path[i_next]];
            double newDist = distance[path[j_prev]][path[i]] + 
                           distance[path[i]][path[j]] + 
                           distance[path[j]][path[i_next]];
            swap(path[i], path[j]);
            return newDist - oldDist;
        }
        else {
            // i和j不相邻
            double oldDist = distance[path[i_prev]][path[i]] + 
                           distance[path[i]][path[i_next]] + 
                           distance[path[j_prev]][path[j]] + 
                           distance[path[j]][path[j_next]];
            double newDist = distance[path[i_prev]][path[j]] + 
                           distance[path[j]][path[i_next]] + 
                           distance[path[j_prev]][path[i]] + 
                           distance[path[i]][path[j_next]];
            swap(path[i], path[j]);
            return newDist - oldDist;
        }
    }
    
    // 更新禁忌表
    void updateTabuTable(int i, int j) {
        // 减少所有表项的禁忌期
        for (auto& row : tabuTable) {
            for (auto& tenure : row) {
                if (tenure > 0) tenure--;
            }
        }
        
        // 设置新的禁忌表项
        tabuTable[i][j] = tabuTenure;
        tabuTable[j][i] = tabuTenure;
    }
    
    // 检查移动是否在禁忌表中
    bool isTabu(int i, int j) {
        return tabuTable[i][j] > 0 || tabuTable[j][i] > 0;
    }
    
    // 禁忌搜索主函数
    vector<int> tabuSearch() {
        // 生成初始解
        vector<int> currentSolution = generateInitialSolution();
        vector<int> bestSolution = currentSolution;
        
        double currentDistance = calculateTotalDistance(currentSolution);
        double bestDistance = currentDistance;
        
        cout << "初始解长度: " << fixed << setprecision(2) << bestDistance << endl;
        
        int noImproveCount = 0;
        random_device rd;
        mt19937 gen(rd());
        uniform_real_distribution<> dis(0.0, 1.0);
        
        // 主迭代循环
        for (int iter = 0; iter < maxIterations; iter++) {
            if (noImproveCount >= maxNoImprove) {
                cout << "达到最大无改进次数，提前终止" << endl;
                break;
            }
            
            double bestNeighborImprovement = 0;
            int bestI = -1, bestJ = -1;
            bool isAspiration = false;
            
            // 搜索邻域
            for (int i = 0; i < n; i++) {
                for (int j = i + 1; j < n; j++) {
                    // 复制当前解进行测试
                    vector<int> neighborSolution = currentSolution;
                    
                    // 执行移动并计算改进
                    double improvement = performSwapMove(neighborSolution, i, j);
                    
                    // 检查是否为最佳邻域解
                    if (improvement < bestNeighborImprovement) {
                        // 如果不在禁忌表中或满足藐视准则
                        if (!isTabu(i, j) || 
                            (calculateTotalDistance(neighborSolution) < bestDistance)) {
                            
                            bestNeighborImprovement = improvement;
                            bestI = i;
                            bestJ = j;
                            
                            // 检查是否满足藐视准则（找到新的全局最优解）
                            if (calculateTotalDistance(neighborSolution) < bestDistance) {
                                isAspiration = true;
                            }
                        }
                    }
                }
            }
            
            // 如果找到改进的移动
            if (bestI != -1 && bestJ != -1) {
                // 执行最佳移动
                performSwapMove(currentSolution, bestI, bestJ);
                currentDistance += bestNeighborImprovement;
                
                // 更新禁忌表
                if (!isAspiration) {
                    updateTabuTable(bestI, bestJ);
                }
                
                // 更新全局最优解
                if (currentDistance < bestDistance) {
                    bestSolution = currentSolution;
                    bestDistance = currentDistance;
                    noImproveCount = 0;
                    
                    if (iter % 100 == 0) {
                        cout << "迭代 " << iter << ": 新最优解长度 = " 
                             << fixed << setprecision(2) << bestDistance << endl;
                    }
                } else {
                    noImproveCount++;
                }
            } else {
                // 如果没有找到改进，随机选择一个非禁忌移动
                bool found = false;
                for (int attempt = 0; attempt < 100 && !found; attempt++) {
                    int i = gen() % n;
                    int j = gen() % n;
                    if (i != j && !isTabu(i, j)) {
                        performSwapMove(currentSolution, i, j);
                        currentDistance = calculateTotalDistance(currentSolution);
                        updateTabuTable(i, j);
                        found = true;
                    }
                }
                
                if (!found) {
                    // 如果找不到非禁忌移动，随机移动
                    int i = gen() % n;
                    int j = gen() % n;
                    while (i == j) j = gen() % n;
                    performSwapMove(currentSolution, i, j);
                    currentDistance = calculateTotalDistance(currentSolution);
                    updateTabuTable(i, j);
                }
                
                noImproveCount++;
            }
            
            // 动态调整禁忌长度
            if (iter % 50 == 0) {
                if (noImproveCount > 50) {
                    tabuTenure = min(tabuTenure + 1, 20 + n / 3);
                } else if (noImproveCount < 10) {
                    tabuTenure = max(tabuTenure - 1, 5);
                }
            }
        }
        
        return bestSolution;
    }
    
    // 输出结果
    void printSolution(const vector<int>& solution) {
        double totalDistance = calculateTotalDistance(solution);
        
        cout << "\n===== 禁忌搜索算法结果 =====" << endl;
        cout << "最优路径长度: " << fixed << setprecision(2) << totalDistance << endl;
        cout << "最优路径: ";
        for (int i = 0; i < min(10, n); i++) {
            cout << solution[i] << " -> ";
        }
        if (n > 10) cout << "... -> " << solution[0];
        else cout << solution[0];
        cout << endl;
        
        // 验证路径
        cout << "\n路径验证:" << endl;
        for (int i = 0; i < n - 1; i++) {
            cout << solution[i] << " -> ";
        }
        cout << solution[n - 1] << " -> " << solution[0] << endl;
    }
};

// 生成随机测试数据
vector<pair<double, double>> generateRandomCities(int n, double maxX = 100, double maxY = 100) {
    vector<pair<double, double>> cities;
    random_device rd;
    mt19937 gen(rd());
    uniform_real_distribution<> disX(0, maxX);
    uniform_real_distribution<> disY(0, maxY);
    
    for (int i = 0; i < n; i++) {
        cities.emplace_back(disX(gen), disY(gen));
    }
    
    return cities;
}

int main() {
    cout << "=== TSP问题禁忌搜索算法求解 ===" << endl;
    
    // 示例：使用预定义城市坐标或随机生成
    int choice;
    cout << "\n选择测试方式:" << endl;
    cout << "1. 使用预定义示例数据（16个城市）" << endl;
    cout << "2. 随机生成数据" << endl;
    cout << "请输入选择 (1-2): ";
    cin >> choice;
    
    vector<pair<double, double>> cities;
    
    if (choice == 1) {
        // 预定义示例数据
        cities = {
            {60, 200}, {180, 200}, {80, 180}, {140, 180},
            {20, 160}, {100, 160}, {200, 160}, {140, 140},
            {40, 120}, {100, 120}, {180, 100}, {60, 80},
            {120, 80}, {180, 60}, {20, 40}, {100, 40}
        };
        cout << "使用预定义16城市数据" << endl;
    } else {
        int n;
        cout << "请输入城市数量: ";
        cin >> n;
        cities = generateRandomCities(n);
        cout << "生成了 " << n << " 个随机城市" << endl;
    }
    
    // 创建TSP问题实例
    TSP tsp(cities);
    
    // 运行禁忌搜索算法
    clock_t start = clock();
    vector<int> bestSolution = tsp.tabuSearch();
    clock_t end = clock();
    
    // 输出结果
    tsp.printSolution(bestSolution);
    
    double runtime = double(end - start) / CLOCKS_PER_SEC;
    cout << "\n计算时间: " << fixed << setprecision(2) << runtime << " 秒" << endl;
    
    return 0;
}