#include <bits/stdc++.h>

using namespace std;

/////////////////////////////////////////////// CLASA GRAF //////////////////////////////////////////////

template<class T = int>
struct Muchie {
    int x, y;
    T cost;

    bool operator < (const Muchie& obj) const {
        return cost < obj.cost;
    }
};

template<class T = int>
class Graph {
private:
    vector<vector<pair<int, T>>> adjList;
    vector<Muchie<T>> edges;
    bool directed;

    // Funtiile pentru Union-Find folosite la determinarea arborelui partial de cost minim (algoritmul lui Kruskal)
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
    /////////////////////////////////////////////////////////

public:
    Graph(int size, bool directed, const vector<Muchie<T>>& edges){       // Constructor 1
        this->edges = edges;
        this->directed = directed;
        this->adjList.resize(size + 1);
        for(auto edge : edges){
            this->adjList[edge.x].push_back({edge.y, edge.cost});
            if(this->directed == false){
                this->adjList[edge.y].push_back({edge.x, edge.cost});
            }
        }
    }
    Graph(bool directed, const vector<vector<pair<int, T>>>& list){       // Constructor 2
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
    vector<pair<int, int>> operator [] (int node) const {        // [] operator overload
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

    // Algoritmul lui Dijkstra pentru drumuri de cost minim intr-un graf, returneaza distanta de la un nod de start startNode la un nod final finalNode
    int dijkstraStartToFinish(int startNode, int finalNode){
        int n = this->adjList.size();
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
            for(auto neigh : this->adjList[node]){
                if(distance[neigh.first] > distance[node] + neigh.second){
                    distance[neigh.first] = distance[node] + neigh.second;
                    minHeap.insert({distance[neigh.first], neigh.first});
                }
            }
        }
        return distance[finalNode];
    }

    // Algoritmul lui Dijkstra pentru drumuri de cost minim intr-un graf, retine in plus traseul parcurs pentru drumul de cost minim
    void dijkstraWithPath(int startNode, vector<T>& distances, int fatherOf[]){
        int n = this->adjList.size();
        vector<bool> vis(n + 1, false);
        set<pair<T, int>> minHeap;
        fatherOf[startNode] = startNode;
        distances[startNode] = 0;
        minHeap.insert({0, startNode});
        while(!minHeap.empty()){
            auto x = (*minHeap.begin());
            int node = x.second;
            minHeap.erase(minHeap.begin());
            if(vis[node]){
                continue;
            }
            vis[node] = true;
            for(auto neigh : this->adjList[node]){
                if(distances[neigh.first] > distances[node] + neigh.second){
                    distances[neigh.first] = distances[node] + neigh.second;
                    fatherOf[neigh.first] = node;
                    minHeap.insert({distances[neigh.first], neigh.first});
                }
            }
        }
    }

    // Algoritmul lui Kruskal pentru arbori partiali de cost minim, returneaza costul unui astfel de arbore
    int getCostFromMST(){
        int n = this->adjList.size();
        int TT[n + 5], RG[n + 5];
        for(int i = 0; i <= n; ++i){
            TT[i] = i;
            RG[i] = 0;
        }
        int result = 0;
        sort(this->edges.begin(), this->edges.end());
        for(auto edge : edges){
            int r1 = doFind(edge.x, TT, RG);
            int r2 = doFind(edge.y, TT, RG);
            if(r1 != r2){
                doUnion(r1, r2, TT, RG);
                result += edge.cost;
            }
        }
        return result;
    }
};


/////////////////////////////////////////////// CLASA SOLUTIE //////////////////////////////////////////////

class Solution {
private:
    // Folosit la problema Lanterna
    struct MuchieLanterna {
        int nod, T, W;
        bool operator < (const MuchieLanterna& obj) const {
            return T > obj.T;
        }
    };
    /////////////////////////////////////////////////////////

    // Functiile Union-Find folosite la problemele Rusuoaica, Bile, Checking-existence-of-edge-length-limited-paths
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

    pair<int, int> doFind(int i, int j, pair<int, int> TT[][255]){
        if(TT[i][j] == make_pair(i, j)){
            return {i, j};
        }
        else{
            return doFind(TT[i][j].first, TT[i][j].second, TT);
        }
    }

    int doUnion(const pair<int, int>& a, const pair<int, int>& b, pair<int, int> TT[][255], int RG[][255]){
        if(RG[a.first][a.second] >= RG[b.first][b.second]){
            TT[b.first][b.second] = a;
            RG[a.first][a.second] += RG[b.first][b.second];
            return RG[a.first][a.second];
        }
        else{
            TT[a.first][a.second] = b;
            RG[b.first][b.second] += RG[a.first][a.second];
            return RG[b.first][b.second];
        }
    }
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////

    // Helper pentru problema Cablaj, calculeaza distanta dintre doua puncte in plan
    double calcDist(const pair<int, int>& a, const pair<int, int>& b){
        return sqrt((b.first - a.first) * (b.first - a.first) + (b.second - a.second) * (b.second - a.second));
    }
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////

    // Helper pentru problema Trilant, afiseaza drumul de cost minim calculat cu algortimul lui Dijkstra pornind de la un node de start startNode
    void findPath(int startNode, int TT[], ofstream& g){
        int cnt = 1, node = startNode;
        while(TT[node] != node){
            cnt++;
            node = TT[node];
        }
        g << cnt << " ";
        while(TT[startNode] != startNode){
            g << startNode << " ";
            startNode = TT[startNode];
        }
        g << startNode << "\n";
    }
    //////////////////////////////////////////////////////////////////

    // Helper pentru problema Lanterna, calculeaza drumul de cost minim pentru o singura lanterna cu algoritmul lui Dijkstra
    int helperLanterna(int lanterna, int n, int k, bool prieten[], const vector<MuchieLanterna> Graf[]){
        int dist[55][1005];
        for(int i = 0; i <= n; ++i){
            for(int j = 0; j <= k; ++j){
                dist[i][j] = INT_MAX;
            }
        }
        int res = INT_MAX;
        priority_queue<MuchieLanterna> minHeap;
        dist[1][lanterna] = 0;
        minHeap.push({1, 0, lanterna});
        while(!minHeap.empty()){
            MuchieLanterna x = minHeap.top();
            minHeap.pop();
            if(x.nod == n){
                res = min(res, x.T);
            }
            for(auto neigh : Graf[x.nod]){
                if(x.W >= neigh.W){
                    int y = x.W - neigh.W;
                    if(prieten[neigh.nod]){
                        y = lanterna;
                    }
                    if(dist[neigh.nod][y] > x.T + neigh.T){
                        dist[neigh.nod][y] = x.T + neigh.T;
                        minHeap.push({neigh.nod, dist[neigh.nod][y], y});
                    }
                }
            }
        }
        return res;
    }
    //////////////////////////////////////////////////////////////////////////////

    // Helper pentru problema Banuti, calculeaza distanta de cost minim pana la un nou rest % bancnota minima si returneaza distanta cea mai mare
    long long helperBanuti(int bancMin, const vector<int>& muchii){
        set<pair<long long, int>> minHeap;
        vector<bool> vis(bancMin, false);
        vector<long long> dist(bancMin, 1e18);
        minHeap.insert({0, 0});
        dist[0] = 0;
        while(!minHeap.empty()){
            auto x = (*minHeap.begin());
            int node = x.second;
            long long cost = x.first;
            minHeap.erase(minHeap.begin());
            if(vis[node]){
                continue;
            }
            vis[node] = true;
            for(auto muchie : muchii){
                int newSum = (node + muchie) % bancMin;
                if(!vis[newSum] && dist[newSum] > cost + muchie){
                    dist[newSum] = cost + muchie;
                    minHeap.insert({dist[newSum], newSum});
                }
            }
        }
        long long maxi = 0;
        for(auto d : dist){
            maxi = max(maxi, d);
        }
        return maxi;
    }

public:
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
        vector<Muchie<>> muchii;

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
        Graph<> Graf(false, graf);
        g << Graf.dijkstraStartToFinish(1, n);
    }
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// https://www.infoarena.ro/problema/trilant
    void trilant(){
        ifstream f("trilant.in");
        ofstream g("trilant.out");

        const int NMAX = 1e5 + 5;
        int n, m, A, B, C, TTA[NMAX], TTB[NMAX], TTC[NMAX];
        vector<vector<pair<int, long long>>> graf;

        f >> n >> m;
        f >> A >> B >> C;
        graf.resize(n + 1);
        for(int i = 1; i <= m; ++i){
            int x, y;
            long long cost;
            f >> x >> y >> cost;
            graf[x].push_back({y, cost});
            graf[y].push_back({x, cost});
        }
        vector<long long> distA(n + 1, LLONG_MAX), distB(n + 1, LLONG_MAX), distC(n + 1, LLONG_MAX);
        for(int i = 1; i <= n; ++i){
            TTA[i] = 0;
            TTB[i] = 0;
            TTC[i] = 0;
        }
        Graph<long long> Graf(false, graf);
        Graf.dijkstraWithPath(A, distA, TTA);
        Graf.dijkstraWithPath(B, distB, TTB);
        Graf.dijkstraWithPath(C, distC, TTC);
        long long minDist = LLONG_MAX;
        int startNode = n;
        for(int i = 1; i <= n; ++i){
            if(distA[i] + distB[i] + distC[i] < minDist){
                minDist = distA[i] + distB[i] + distC[i];
                startNode = i;
            }
        }
        g << minDist << "\n";
        findPath(startNode, TTA, g);
        findPath(startNode, TTB, g);
        findPath(startNode, TTC, g);
    }
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// https://www.infoarena.ro/problema/dragoni
    void dragoni(){
        ifstream f("dragoni.in");
        ofstream g("dragoni.out");

        const int NMAX = 805;
        int cer, n, m, dragoni[NMAX], dist[NMAX][NMAX];
        vector<pair<int, int>> graf[NMAX];

        f >> cer;
        f >> n >> m;
        for(int i = 1; i <= n; ++i){
            f >> dragoni[i];
        }
        for(int i = 1; i <= m; ++i){
            int x, y, c;
            f >> x >> y >> c;
            graf[x].push_back({c, y});
            graf[y].push_back({c, x});
        }
        if(cer == 1){
            queue<int> q;
            vector<bool> vis(n + 1, false);
            vis[1] = true;
            int res = dragoni[1];
            q.push(1);
            while(!q.empty()){
                int x = q.front();
                q.pop();
                res = max(res, dragoni[x]);
                for(auto neigh : graf[x]){
                    if(dragoni[1] >= neigh.first && !vis[neigh.second]){
                        vis[neigh.second] = true;
                        q.push(neigh.second);
                    }
                }
            }
            g << res;
        }
        else{
            set<pair<pair<int, int>, int>> minHeap;
            for(int i = 1; i <= n; ++i){
                for(int j = 1; j <= n; ++j){
                    dist[i][j] = INT_MAX;
                }
            }
            minHeap.insert({{0, 1}, 1});
            dist[1][1] = 0;
            while(!minHeap.empty()){
                pair<pair<int, int>, int> x = (*minHeap.begin());
                int nod = x.first.second;
                int currDrag = x.second;
                minHeap.erase(minHeap.begin());
                for(auto neigh : graf[nod]){
                    int nextDrag = currDrag;
                    if(dragoni[neigh.second] > dragoni[currDrag]){
                        nextDrag = neigh.second;
                    }
                    if(dist[neigh.second][nextDrag] > dist[nod][currDrag] + neigh.first && dragoni[currDrag] >= neigh.first){
                        dist[neigh.second][nextDrag] = dist[nod][currDrag] + neigh.first;
                        minHeap.insert({{dist[neigh.second][nextDrag], neigh.second}, nextDrag});
                    }
                }
            }
            int res = INT_MAX;
            for(int j = 1; j <= n; ++j){
                res = min(res, dist[n][j]);
            }
            g << res;
        }
    }
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// https://www.infoarena.ro/problema/oracol
    void oracol(){
        ifstream f("oracol.in");
        ofstream g("oracol.out");

        int n;
        vector<Muchie<>> muchii;

        f >> n;
        for(int i = 1; i <= n; ++i){
            for(int j = i; j <= n; ++j){
                int x;
                f >> x;
                muchii.push_back({i - 1, j, x});
            }
        }
        Graph<> Graf(n + 1, false, muchii);
        g << Graf.getCostFromMST();
    }
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// https://www.infoarena.ro/problema/bile
    void bile(){
        ifstream f("bile.in");
        ofstream g("bile.out");

        const int NMAX = 255;
        int n;
        pair<int, int> coordBile[NMAX * NMAX], TT[NMAX][NMAX];
        int RG[NMAX][NMAX];
        bool vis[NMAX][NMAX];
        vector<pair<int, int>> d = { {-1, 0}, {0, 1}, {1, 0}, {0, -1} };

        f >> n;
        for(int i = 1; i <= n * n; ++i){
            f >> coordBile[i].first >> coordBile[i].second;
        }
        for(int i = 1; i <= n; ++i){
            for(int j = 1; j <= n; ++j){
                TT[i][j] = {i, j};
                RG[i][j] = 1;
            }
        }
        vector<int> res;
        int maxi = 1;
        res.push_back(0);
        vis[coordBile[n * n].first][coordBile[n * n].second] = true;
        for(int i = n * n - 1; i >= 1; --i){
            vis[coordBile[i].first][coordBile[i].second] = true;
            res.push_back(maxi);
            for(auto dir : d){
                int in = coordBile[i].first + dir.first;
                int jn = coordBile[i].second + dir.second;
                if((in >= 1 && in <= n && jn >= 1 && jn <= n) && vis[in][jn]){
                    pair<int, int> r1 = doFind(coordBile[i].first, coordBile[i].second, TT), r2 = doFind(in, jn, TT);
                    if(r1 != r2){
                        maxi = max(maxi, doUnion(r1, r2, TT, RG));
                    }
                }
            }
        }
        reverse(res.begin(), res.end());
        for(auto i : res){
            g << i << "\n";
        }
    }
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// https://leetcode.com/problems/checking-existence-of-edge-length-limited-paths/
    vector<bool> distanceLimitedPathsExist(int n, vector<vector<int>>& edgeList, vector<vector<int>>& queries) {
        const int NMAX = 1e5 + 5;
        int TT[NMAX], RG[NMAX];
        for(int i = 0; i < n; ++i){
            TT[i] = i;
            RG[i] = 0;
        }
        sort(edgeList.begin(), edgeList.end(), [&](const vector<int>& a, const vector<int>& b){
            return a[2] < b[2];
        });
        vector<bool> res(queries.size(), false);
        for(int i = 0; i < queries.size(); ++i){
            queries[i].push_back(i);
        }
        sort(queries.begin(), queries.end(), [&](const vector<int>& a, const vector<int>& b){
            return a[2] < b[2];
        });
        int i = 0;
        for(int j = 0; j < queries.size(); ++j){
            while(i < edgeList.size() && edgeList[i][2] < queries[j][2]){
                doUnion(edgeList[i][0], edgeList[i][1], TT, RG);
                i++;
            }
            int r1 = doFind(queries[j][0], TT, RG), r2 = doFind(queries[j][1], TT, RG);
            if(r1 == r2){
                res[queries[j][3]] = true;
            }
        }
        return res;
    }
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// https://www.infoarena.ro/problema/lanterna
    void lanterna(){
        ifstream f("lanterna.in");
        ofstream g("lanterna.out");

        const int NMAX = 55;
        int n, k, m;
        vector<MuchieLanterna> Graf[NMAX];
        bool prieten[NMAX];

        f >> n >> k;
        for(int i = 1; i <= n; ++i){
            f >> prieten[i];
        }
        f >> m;
        for(int i = 1; i <= m; ++i){
            int x, y, T, W;
            f >> x >> y >> T >> W;
            Graf[x].push_back({y, T, W});
            Graf[y].push_back({x, T, W});
        }
        int res = helperLanterna(k, n, k, prieten, Graf);
        int st = 1, dr = k, l = k;
        while(st <= dr){
            int mij = (st + dr) / 2;
            if(helperLanterna(mij, n, k, prieten, Graf) == res){
                l = mij;
                dr = mij - 1;
            }
            else{
                st = mij + 1;
            }
        }
        g << res << " " << l;
    }
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// https://www.infoarena.ro/problema/banuti
    void banuti(){
        ifstream f("banuti.in");
        ofstream g("banuti.out");

        const int INF = 1e9;
        const int NMAX = 5e4 + 5;
        int n, banuti[NMAX], bancMin = INF;
        vector<int> muchiiInitial, muchii;

        f >> n;
        for(int i = 1; i <= n; ++i){
            f >> banuti[i];
            bancMin = min(bancMin, banuti[i]);
        }
        muchiiInitial.resize(bancMin, INF);
        for(int i = 1; i <= n; ++i){
            muchiiInitial[banuti[i] % bancMin] = min(muchiiInitial[banuti[i] % bancMin], banuti[i]);
        }
        for(int i = 0; i < muchiiInitial.size(); ++i){
            if(muchiiInitial[i] != INF){
                muchii.push_back(muchiiInitial[i]);
            }
        }
        long long res = helperBanuti(bancMin, muchii);
        if(res != 1e18){
            g << res - bancMin;
        }
        else{
            g << -1;
        }
    }
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

};

/////////////////////////////////////////////// CLASA SOLUTIE //////////////////////////////////////////////


int main(){
    
}