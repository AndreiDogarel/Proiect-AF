#include <bits/stdc++.h>

using namespace std;

/////////////////////////////////////////////// CLASA GRAF //////////////////////////////////////////////
class Graph {
private:
    vector<vector<int>> adjList;
    bool directed;

    void criticalConnectionsDfs(int curr, int prev, vector<int> &vis, vector<int> &disc, vector<int> &low, vector<vector<int>> &bridges, int timer){        // Helper for the function below
        vis[curr] = 1;
        disc[curr] = low[curr] = timer;
        timer++;
        for(auto node : this->adjList[curr]){
            if(node == prev){
                continue;
            }
            if(!vis[node]){
                criticalConnectionsDfs(node, curr, vis, disc, low, bridges, timer);
                low[curr] = min(low[node], low[curr]);

                if(low[node] > disc[curr]){
                    vector<int> sol;
                    sol.push_back(node);
                    sol.push_back(curr);
                    bridges.push_back(sol);
                }
            }
            else{
                low[curr] = min(low[node], low[curr]);
            }
        }
    }

public:
    Graph(int size, bool directed, const vector<vector<int>> &edges){       // Constructor 1
        this->directed = directed;
        this->adjList.resize(size + 1);
        for(auto edge : edges){
            this->adjList[edge[0]].push_back(edge[1]);
            if(this->directed == false){
                this->adjList[edge[1]].push_back(edge[0]);
            }
        }
    }
    Graph(bool directed, const vector<vector<int>>& list){      // Constructor 2
        this->directed = directed;
        this->adjList = list;
    }
    Graph(const Graph& obj){        // Copy constructor
        this->adjList = obj.adjList;
        this->directed = obj.directed;
    }
    Graph& operator = (const Graph& obj){       // = operator overload
        this->adjList = obj.adjList;
        this->directed = obj.directed;
    }
    vector<int>& operator [] (int node){        // [] operator overload
        try{
            if(node > this->adjList.size() || node < 0){
                throw runtime_error("Nod invalid!");
            }
            return this->adjList[node];
        }
        catch(const exception& e){
            cout << e.what() << "\n";
        }
    }

    bool checkForBipartition(int startNode, vector<int>& colours){      // Returns true if the graph is bipartite, else returns false
        queue<int> q;
        q.push(startNode);
        colours[startNode] = 1;
        while(!q.empty()){
            int x = q.front();
            q.pop();
            for(auto i : this->adjList[x]){
                if(colours[i] == -1){
                    colours[i] = 1 - colours[x];
                    q.push(i);
                }
                else if(colours[i] == colours[x]){
                    return false;
                }
            }
        }
        return true;
    }

    vector<int> findBipartition(int startNode){
        vector<int> colours(this->adjList.size() + 1, -1);
        queue<int> q;
        q.push(startNode);
        colours[startNode] = 1;
        while(!q.empty()){
            int x = q.front();
            q.pop();
            for(auto i : this->adjList[x]){
                if(colours[i] == -1){
                    colours[i] = 1 - colours[x];
                    q.push(i);
                }
            }
        }
        return colours;
    }

    vector<int> findTopologicalSort(){      // Returns a topological sort solution for a directed graph
        const int n = this->adjList.size();
        vector<int> intDeg(n, 0), res;
        for(auto neighbours : this->adjList){
            for(auto neighbour : neighbours){
                intDeg[neighbour]++;
            }
        }
        queue<int> q;
        for(int i = 0; i < n; ++i){
            if(!intDeg[i]){
                q.push(i);
            }
        }
        while(!q.empty()){
            int x = q.front();
            q.pop();
            res.push_back(x);
            for(auto i : adjList[x]){
                intDeg[i]--;
                if(!intDeg[i]){
                    q.push(i);
                }
            }
        }
        return res;
    }

    vector<vector<int>> criticalConnections(){      // Returns critical connections in a graph
        int n = this->adjList.size();
        vector<int> disc(n), low(n), vis(n);
        vector<vector<int>> bridges;
        int timer = 1;
        criticalConnectionsDfs(0, -1, vis, disc, low, bridges, timer);
        return bridges;
    }
};
/////////////////////////////////////////////// CLASA GRAF //////////////////////////////////////////////


/////////////////////////////////////////////// CLASA SOLUTIE //////////////////////////////////////////////
class Solution {
private:
    void findIsland(int i, int j, int n, vector<vector<bool>>& vis, vector<pair<int, int>>& island, vector<vector<int>>& grid){     // Helper for shortestBridge
        queue<pair<int, int>> q;
        vector<pair<int, int>> directions = 
        {
            {-1, 0}, {0, 1}, {1, 0}, {0, -1}
        };
        q.push({i, j});
        vis[i][j] = true;
        island.push_back({i, j});
        while(!q.empty()){
            pair<int, int> x = q.front();
            q.pop();
            for(auto d : directions){
                int newX = d.first + x.first;
                int newY = d.second + x.second;
                if(newX >= 0 && newX < n && newY >= 0 && newY < n && !vis[newX][newY] && grid[newX][newY]){
                    vis[newX][newY] = true;
                    q.push({newX, newY});
                    island.push_back({newX, newY});
                }
            }
        }
    }

    int doFind(int n, int TT[], int RG[]){
        if(TT[n] == n){
            return n;
        }
        else{
            return doFind(TT[n], TT, RG);
        }
    }

    void doUnion(int a, int b, int TT[], int RG[]){
        int tta = doFind(a, TT, RG);
        int ttb = doFind(b, TT, RG);
        if(tta != ttb){
            if(RG[tta] >= RG[ttb]){
                TT[ttb] = tta;
                RG[tta] += RG[ttb];
            }
            else{
                TT[tta] = ttb;
                RG[ttb] += RG[tta];
            }
        }
    }

public:
// https://leetcode.com/problems/possible-bipartition/
    bool possibleBipartition(int n, vector<vector<int>>& dislikes) {
        Graph G(n, false, dislikes);
        vector<int> colours(n + 1, -1);
        for(int i = 1; i <= n; ++i){
            if(colours[i] == -1){
                if(!G.checkForBipartition(i, colours)){
                    return false;
                }
            }
        }
        return true;
    }
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// https://leetcode.com/problems/course-schedule-ii/
    vector<int> findOrder(int numCourses, vector<vector<int>>& prerequisites) {
        Graph G(numCourses, true, prerequisites);
        vector<int> res = G.findTopologicalSort();
        reverse(res.begin(), res.end());
        if(res.size() < numCourses){
            res.clear();
        }
        return res;
    }
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// https://leetcode.com/problems/critical-connections-in-a-network/
    vector<vector<int>> criticalConnections(int n, vector<vector<int>>& connections) {
        Graph G(n, false, connections);
        return G.criticalConnections();
    }
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// https://leetcode.com/problems/find-eventual-safe-states/
    vector<int> eventualSafeNodes(vector<vector<int>>& graph) {
        int n = graph.size();
        vector<vector<int>> oppGraph;
        oppGraph.resize(n);
        for(int i = 0; i < n; ++i){
            for(auto neigh : graph[i]){
                oppGraph[neigh].push_back(i);
            }
        }
        Graph G(true, oppGraph);
        vector<int> res = G.findTopologicalSort();
        sort(res.begin(), res.end());
        return res;
    }
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// https://leetcode.com/problems/shortest-bridge/
    int shortestBridge(vector<vector<int>>& grid) {
        int n = grid.size();
        vector<pair<int, int>> firstIsland, secondIsland;
        vector<vector<bool>> vis(n, vector<bool>(n, false));
        int cnt = 0;
        for(int i = 0; i < n; ++i){
            for(int j = 0; j < n; ++j){
                if(grid[i][j] && !vis[i][j]){
                    if(!cnt){
                        findIsland(i, j, n, vis, firstIsland, grid);
                        cnt++;
                    }
                    else{
                        findIsland(i, j, n, vis, secondIsland, grid);
                    }
                }
            }
        }
        int res = 1e9;
        for(auto i1 : firstIsland){
            for(auto i2 : secondIsland){
                res = min(res, abs(i1.first - i2.first) + abs(i1.second - i2.second) - 1);
            }
        }
        return res;
    }
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// https://leetcode.com/problems/satisfiability-of-equality-equations/
    bool equationsPossible(vector<string>& equations) {
        int TT[30];
        int RG[30];
        for(char i = 'a'; i <= 'z'; ++i){
            TT[i - 'a'] = i - 'a';
            RG[i - 'a'] = 1;
        }
        for(auto eq : equations){
            if(eq[1] == '='){
                doUnion(eq[0] - 'a', eq[3] - 'a', TT, RG);
            }
        }
        for(auto eq : equations){
            if(eq[1] == '!'){
                if(doFind(eq[0] - 'a', TT, RG) == doFind(eq[3] - 'a', TT, RG)){
                    return false;
                }
            }
        }
        return true;
    }
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// https://www.infoarena.ro/problema/padure
    int minCostForest(vector<vector<int>>& grid, const int& n, const int& m, const int& pl, const int& pc, const int& cl, const int& cc){
        vector<vector<int>> cost;
        vector<pair<int, int>> directions = { {-1, 0}, {0, 1}, {1, 0}, {0, -1} };
        cost.assign(n, vector<int>(m, 1e9));
        deque<pair<int, int>> dq;
        cost[pl][pc] = 0;
        dq.push_front({pl, pc});
        while(!dq.empty()){
            int x = dq.front().first;
            int y = dq.front().second;
            dq.pop_front();
            for(auto d : directions){
                int inou = d.first + x;
                int jnou = d.second + y;
                if(inou >= 0 && inou < n && jnou >= 0 && jnou < m && cost[inou][jnou] > cost[x][y]){
                    if(grid[inou][jnou] != grid[x][y]){
                        cost[inou][jnou] = cost[x][y] + 1;
                        dq.push_back({inou, jnou});
                    }
                    else{
                        cost[inou][jnou] = cost[x][y];
                        dq.push_front({inou, jnou});
                    }
                }
            }
        }
        return cost[cl][cc];
    }
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// https://codeforces.com/contest/1144/problem/F
    string graphWithoutLongDirectedPaths(int n, vector<vector<int>>& connections){
        string ans = "";
        Graph G(n, false, connections);
        vector<int> colours(n + 1, -1);
        if(!G.checkForBipartition(1, colours)){
            return "NO";
        }
        else{
            colours = G.findBipartition(1);
            for(int i = 0; i < connections.size(); ++i){
                if(colours[connections[i][0]] == 0 && colours[connections[i][1]] == 1){
                    ans += "0";
                }
                else{
                    ans += "1";
                }
            }
            return ("YES\n" + ans);
        }
    }
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// https://codeforces.com/contest/1881/problem/F
    int minimumMaximumDistance(int testCases){
        
    }
};
/////////////////////////////////////////////// CLASA SOLUTIE //////////////////////////////////////////////


int main(){
    
}