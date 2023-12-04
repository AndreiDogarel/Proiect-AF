#include <bits/stdc++.h>

using namespace std;

/////////////////////////////////////////////// CLASA GRAF //////////////////////////////////////////////

class Graph {
private:
    vector<vector<int>> adjList;
    vector<vector<pair<int, int>>> adjListCosts;
    bool directed;

    // Helper pentru functia ce returneaza muchiile critice, calculeaza timpul de descoperire si cel mai mic nivel in care poate ajunge un nod 
    // printr-o muchie de intoarcere din arborele DFS
    void criticalConnectionsDfs(int curr, int prev, vector<int> &vis, vector<int> &disc, vector<int> &low, vector<vector<int>> &bridges, int timer){
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
    Graph(int size, bool directed, const vector<vector<int>>& edges){       // Constructor 1
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
    Graph(bool directed, const vector<vector<pair<int, int>>>& list){       // Constructor 3
        this->directed = directed;
        this->adjListCosts = list;
    }
    Graph(const Graph& obj){        // Copy constructor
        this->adjList = obj.adjList;
        this->directed = obj.directed;
    }
    Graph& operator = (const Graph& obj){       // = operator overload
        this->adjList = obj.adjList;
        this->directed = obj.directed;
    }
    vector<int> operator [] (int node) const {        // [] operator overload
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

    bool checkForBipartition(int startNode, vector<int>& colours){      // Returneaza TRUE daca graful este bipartit, FALSE altfel
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

    vector<int> findBipartition(int startNode){     // Returneaza o posibilia colorare a unui graf bipartit
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

    vector<int> findTopologicalSort(){      // Returneaza o posibila solutie pentru sortarea topologica a unui graf orientat
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

    vector<vector<int>> criticalConnections(){      // Returneaza un vector de muchh critice intr-un graf
        int n = this->adjList.size();
        vector<int> disc(n), low(n), vis(n);
        vector<vector<int>> bridges;
        int timer = 1;
        criticalConnectionsDfs(0, -1, vis, disc, low, bridges, timer);
        return bridges;
    }

    int dijkstra(int startNode, int finalNode){
        int n = this->adjListCosts.size();
        set<pair<int, int>> minHeap;
        vector<bool> vis(n + 1, false);
        vector<int> distance(n + 1, INT_MAX);
        distance[startNode] = 0;
        minHeap.insert({0, startNode});
        while(!minHeap.empty()){
            int node = (*minHeap.begin()).second;
            minHeap.erase(minHeap.begin());
            if(vis[node]){
                continue; 
            }
            vis[node] = true;
            for(auto neigh : this->adjListCosts[node]){
                if(distance[neigh.first] > distance[node] + neigh.second){
                    distance[neigh.first] = distance[node] + neigh.second;
                    minHeap.insert({distance[neigh.first], neigh.first});
                }
            }
        }
        return distance[finalNode];
    }
};

/////////////////////////////////////////////// CLASA GRAF //////////////////////////////////////////////


struct Muchie {
    int x, y, cost;
    bool operator < (const Muchie& obj){
        return cost < obj.cost;
    }
};


/////////////////////////////////////////////// CLASA SOLUTIE //////////////////////////////////////////////

class Solution {
private:
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Helper pentru problema Shortest Bridge, marcheaza zonele de 1 invecinate ce formeaza o insula
    void findIsland(int i, int j, int n, vector<vector<bool>>& vis, vector<pair<int, int>>& island, vector<vector<int>>& grid){
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
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Helper pentru problema Satisfiability-Of-Equality-Equations, uneste radacini de paduri de elemente dupa rang
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

    double calcDist(const pair<int, int>& a, const pair<int, int>& b){
        return sqrt((b.first - a.first) * (b.first - a.first) + (b.second - a.second) * (b.second - a.second));
    }
/////////////////////////////////////////////////////////////// TEMA 1 ////////////////////////////////////////////////////////////////////////
// Helper pentru problema Minimum Maximum Distance, calculeaza distanta de la nodul de plecare la fiecare nod in parte
    void calcDistances(const Graph& G, int curr, int prev, vector<int>& d){
        d[curr] = d[prev] + 1;
        for(auto v : G[curr]){
            if(v != prev){
                calcDistances(G, v, curr, d);
            }
        }
    }
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Helper pentru problema Patrol2, calculeaza timpul minim de a ajunge de la nodul node la fiecare nod in parte in functie de disponibilitatea la timpul time
    void escapePatrols(int node, vector<vector<int>>& minimumTime, vector<vector<bool>>& occupied, const int fullCycle, const Graph& G){
        queue<pair<int, int>> q;
        q.push({node, 0});
        minimumTime[node][0] = 0;
        while(!q.empty()){
            pair<int, int> x = q.front();
            q.pop();
            int time = (x.second + 1) % fullCycle;
            for(auto neigh : G[x.first]){
                if(!occupied[neigh][time] && minimumTime[neigh][time] > minimumTime[x.first][x.second] + 1){
                    minimumTime[neigh][time] = minimumTime[x.first][x.second] + 1;
                    q.push({neigh, time});
                }
            }
            if(!occupied[x.first][time] && minimumTime[x.first][time] > minimumTime[x.first][x.second] + 1){
                minimumTime[x.first][time] = minimumTime[x.first][x.second] + 1;
                q.push({x.first, time});
            }
        }
    }
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

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
    int minimumMaximumDistance(int n, vector<int>& marked, vector<vector<int>>& connections){
        Graph G(n, false, connections);
        if(marked.size() == 1){
            return 0;
        }
        vector<int> d1(n + 1, 0), d2(n + 1, 0);
        d1[0] = d2[0] = -1;
        calcDistances(G, marked[0], 0, d1);
        int poz = marked[0];
        for(auto node : marked){
            if(d1[node] > d1[poz]){
                poz = node;
            }
        }
        calcDistances(G, poz, 0, d2);
        poz = marked[0];
        for(auto node : marked){
            if(d2[node] > d2[poz]){
                poz = node;
            }
        }
        return (d2[poz] + 1) / 2;
    }
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// https://www.infoarena.ro/problema/patrol2
    int bestEscapeTimePatrols(int n, vector<vector<int>>& connections, int numberOfPatrols, vector<int>& patrolLength, vector<vector<int>>& patrols){
        Graph G(n, false, connections);
        const int fullCycle = 420;
        vector<vector<bool>> occupied(n, vector<bool>(fullCycle, false));
        vector<vector<int>> minimumTime(n, vector<int>(fullCycle, 1e9));
        for(int i = 0; i < fullCycle; ++i){
            for(int j = 0; j < numberOfPatrols; ++j){
                occupied[patrols[j][i % patrolLength[j]]][i] = true;
            }
        }
        escapePatrols(0, minimumTime, occupied, fullCycle, G);
        int res = 1e9;
        for(int i = 0; i < fullCycle; ++i){
            res = min(res, minimumTime[n - 1][i]);
        }
        if(res == 1e9){
            return -1;
        }
        else{
            return res;
        }
    }
///////////////////////////////////////////////////////////////////// TEMA 1 //////////////////////////////////////////////////////////////////


///////////////////////////////////////////////////////////////////// TEMA 2 //////////////////////////////////////////////////////////////////
// https://www.infoarena.ro/problema/cablaj
    void cablaj(){
        ifstream f("cablaj.in");
        ofstream g("cablaj.out");

        const double INF = 1e9;
        int n;
        vector<pair<int, int>> points;

        f >> n;
        for(int i = 1; i <= n; ++i){
            int x, y;
            f >> x >> y;
            points.push_back({x, y});
        }
        vector<double> dist(n, INF);
        vector<bool> vis(n, false);
        vis[0] = true;
        int currVf = 0;
        int nrMuchii = 0;
        double res = 0.0;
        while(nrMuchii < n - 1){
            for(int i = 0; i < n; ++i){
                if(!vis[i]){
                    dist[i] = min(dist[i], calcDist(points[i], points[currVf]));
                }
            }
            double minDist = INF;
            for(int i = 0; i < n; ++i){
                if(!vis[i] && minDist > dist[i]){
                    minDist = dist[i];
                    currVf = i;
                }
            }
            vis[currVf] = true;
            res += minDist;
            nrMuchii++;
        }
        g << fixed << setprecision(4) << res;
    }
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// https://www.infoarena.ro/problema/rusuoaica
    void rusuoaica(){
        ifstream f("rusuoaica.in");
        ofstream g("rusuoaica.out");

        const int NMAX = 1e5 + 5;
        int n, m, A, TT[NMAX], RG[NMAX];
        vector<Muchie> muchii;

        f >> n >> m >> A;
        for(int i = 1; i <= m; ++i){
            int x, y, cost;
            f >> x >> y >> cost;
            muchii.push_back({x, y, cost});
        }
        for(int i = 1; i <= n; ++i){
            TT[i] = i;
            RG[i] = 1;
        }
        sort(muchii.begin(), muchii.end());
        int res = 0, nr = 0;
        for(int i = 0; i < m && nr < n - 1; ++i){
            int r1 = doFind(muchii[i].x, TT, RG), r2 = doFind(muchii[i].y, TT, RG);
            if(r1 != r2 && muchii[i].cost <= A){
                doUnion(r1, r2, TT, RG);
                res += muchii[i].cost;
                nr++;
            }
            else{
                res -= muchii[i].cost;
            }
        }
        g << res + (n - nr - 1) * A;
    }
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// https://www.infoarena.ro/problema/camionas
    void camionas(){
        ifstream f("camionas.in");
        ofstream g("camionas.out");

        vector<vector<pair<int, int>>> graf;
        int n, m, G;
        f >> n >> m >> G;
        graf.resize(n + 1);
        for(int i = 1; i <= m; ++i){
            int x, y, c;
            f >> x >> y >> c;
            if(c >= G){
                c = 0;
            }
            else{
                c = 1;
            }
            graf[x].push_back({y, c});
            graf[y].push_back({x, c});
        }
        Graph Graf(false, graf);
        g << Graf.dijkstra(1, n);
    }
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

};

/////////////////////////////////////////////// CLASA SOLUTIE //////////////////////////////////////////////


int main(){
    Solution s;
    s.camionas();
    return 0;
}