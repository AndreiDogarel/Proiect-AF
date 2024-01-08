#include <bits/stdc++.h>

using namespace std;


class Graph {
private:
    vector<vector<int>> adjList;
    vector<vector<int>> capacity, flow;
    vector<int> parent;
    vector<vector<bool>> hasEdge;
    vector<bool> visited;

    int BFS(int start, int finish);     // Functie BFS ajutatoare pentru algoritmul de flux maxim 

public:
    // Constructori
    Graph(int size){
        adjList.resize(size + 5);
        capacity.resize(size + 5, vector<int>(size + 5, 0));
        flow.resize(size + 5, vector<int>(size + 5, 0));
        parent.resize(size + 5);
        hasEdge.resize(size + 5, vector<bool>(size + 5, false));
        visited.resize(size + 5, false);
    }

    // Getters & Setters
    vector<vector<int>> getFlow() const { return this->flow; }
    vector<vector<int>> getCapacity() const { return this->capacity; }
    void setFlow(int x, int y, int value){ this->flow[x][y] += value; }
    void setCapacity(int x, int y, int value){ this->capacity[x][y] += value; }

    // Operatori
    vector<int>& operator [] (int index){
        try{
            if(index > this->adjList.size() || index < 0){
                throw runtime_error("Index invalid!");
            }
            return this->adjList[index];
        }
        catch(const exception& e){
            cout << e.what() << "\n";
        }
    }

    // Metode
    bool verifyEdge(int x, int y) const { return this->hasEdge[x][y]; }     // Verifica daca exista o muchie de la x la y

    bool verifyIfVisited(int node) const { return this-> visited[node]; }       // Verifica daca un nod a fost vizitat sau nu

    void add(int x, int y, int capacity){       // Adauga muchia (x, y) cu o capacitate
        this->adjList[x].push_back(y);
        this->adjList[y].push_back(x);
        this->capacity[x][y] = capacity;
        this->hasEdge[x][y] = true;
        this->hasEdge[y][x] = true;
    }

    int maxFlow(int start, int finish);

    void findMinimumVertexCover(int node);
};

int Graph::BFS(int start, int finish){      // Algoritm BFS care cauta un drum de crestere
    fill(this->visited.begin(), this->visited.end(), false);
    queue<int> q;
    this->visited[start] = 1;
    q.push(start);
    while(!q.empty()){
        int node = q.front();
        q.pop();
        if(node == finish){
            break;
        }
        for(auto it : this->adjList[node]){     // Verificam toti vecinii nodului curent
            if(!this->visited[it] && this->capacity[node][it] > this->flow[node][it]){      // Daca gasim un vecin cu o muchie nesaturata, il introducem in coada
                this->visited[it] = 1;
                parent[it] = node;
                q.push(it);
            }
        }
    }
    return this->visited[finish];
}

int Graph::maxFlow(int start, int finish){      // Functia returneaza fluxul maxim din graf
    int flow = 0;
    while(BFS(start, finish)){      // Cautam un drum de crestere
        for(int it : this->adjList[finish]){
            if(this->visited[it] && capacity[it][finish] > this->flow[it][finish]){     // Exista un vecin cu o muchie nesaturata
                int node = finish, currFlux = INT_MAX;
                parent[finish] = it;
                while(node != start){       // Aflam fluxul cu care o sa actualizam drumul de crestere, daca fluxul este 0, ne oprim
                    int prev = parent[node];
                    currFlux = min(currFlux, this->capacity[prev][node] - this->flow[prev][node]);
                    node = prev;
                }
                if(currFlux == 0){
                    continue;
                }
                node = finish;
                while(node != start){       // Actualizam drumul de crestere
                    int prev = parent[node];
                    this->flow[prev][node] += currFlux;
                    this->flow[node][prev] -= currFlux;
                    node = prev;
                }
                flow += currFlux;
            }
        }
    }
    return flow;
}

void Graph::findMinimumVertexCover(int node){       // Functia parcurge graful folosind un algoritm DFS si marcheaza toate nodurile vizitate incepand din node
    this->visited[node] = true;
    for(int next : this->adjList[node]){
        if(!this->visited[next] && this->capacity[node][next] > this->flow[node][next]){
            findMinimumVertexCover(next);
        }
    }
}


class Solution {
private:
    // Helper pentru problema Find Itinerary, parcurge graful si construieste un ciclu Eulerian 
    void findItineraryHelper(string start, vector<string>& sol, unordered_map<string, vector<string>>& Graph){
        while(!Graph[start].empty()){
            string next = Graph[start].back();
            Graph[start].pop_back();       // Eliminam muchia, deoarece intr-un ciclu Euler fiecare muchie apare o singura data
            findItineraryHelper(next, sol, Graph);
        }
        sol.push_back(start);
    }

    // Helper pentru problema Valid Arrangement, parcurge graful si construieste un ciclu Eulerian
    void validArrangementHelper(int start, vector<vector<int>>& sol, unordered_map<int, vector<int>>& Graph){
        while(!Graph[start].empty()){
            int next = Graph[start].back();
            Graph[start].pop_back();
            validArrangementHelper(next, sol, Graph);
            sol.push_back({start, next});
        }
    }

public:
    // https://www.infoarena.ro/problema/harta
    void Harta(){
        ifstream fin("harta.in");
        ofstream fout("harta.out");

        int n, nrMuchii = 0;
        fin >> n;
        int s = 0, t = 2 * n + 1;
        Graph Graf(t);
        for(int i = 1; i <= n; ++i){
            int x, y;
            fin >> x >> y;
            nrMuchii += x;
            for(int j = 1; j <= n; ++j){
                if(j != i){
                    Graf.add(i, j + n, 1);      // Dublam numarul de noduri si trasam o muchie de capacitate 1 de la un nod la fiecare copie, cu exceptia copiei sale
                }
            }
            Graf.add(s, i, x);      // Cream doua noduri imaginare s si t si trasam muchii de capacitate x de la s la primele n noduri si muchii de capacitate y de la cele n copii la t
            Graf.add(i + n, t, y);
        }
        Graf.maxFlow(s, t);     // Facem flux maxim pe acel graf
        fout << nrMuchii << "\n";
        for(int i = 1; i <= n; ++i){
            for(int j = n + 1; j < t; ++j){
                if(Graf.getFlow()[i][j]){       // Daca muchia (i, j) cu 1 <= i <= n si (n + 1) <= j <= t este saturata, atunci constituie o solutie 
                    fout << i << " " << j - n << "\n";
                }
            }
        }
    }

    // https://codeforces.com/problemset/problem/546/E
    void SoldierAndTraveling(){
        ios_base::sync_with_stdio(false);
        cin.tie(nullptr);

        int n, m, sumaA = 0, sumaB = 0;
        cin >> n >> m;
        int s = 0, t = 2 * n + 1;
        Graph Graf(t);
        for(int i = 1; i <= n; ++i){    // Se creeaza un nod imaginar de start s si se traseaza muchii de capacitate a[i] catre primele n noduri, reprezentand numarul de soldati existenti in orasul i 
            int x;
            cin >> x;
            Graf.add(s, i, x);
            sumaA += x;
        }
        for(int i = 1; i <= n; ++i){
            int x;
            cin >> x;
            Graf.add(i + n, t, x);      // Se creeaza un nod imaginar de final t si se traseaza muchii de capacitate b[i] catre urmatoarele n noduri, reprezentand numarul de soldati prezenti in orasul i dupa ce s-au mutat 
            sumaB += x;
        }
        for(int i = 1; i <= m; ++i){
            int x, y;
            cin >> x >> y;
            Graf.add(x, y + n, INT_MAX);
            Graf.add(y, x + n, INT_MAX);    // Se creeaza muchii de capacitate infinita intre noduri si copiile lor
        }
        for(int i = 1; i <= n; ++i){
            Graf.add(i, i + n, INT_MAX);
        }
        int res = Graf.maxFlow(s, t);   // Facem flux maxim pe graf
        if(res == sumaA && sumaA == sumaB){     // Ca sa existe o solutie trebuie ca suma soldatilor din toate cele n orase sa fie egala cu suma soldatilor din cele n orase dupa mutare
            cout << "YES\n";
            for(int i = 1; i <= n; ++i){
                for(int j = n + 1; j <= 2 * n; ++j){
                    cout << Graf.getFlow()[i][j] << " ";
                }
                cout << "\n";
            } 
        }
        else{
            cout << "NO";   // Altfel afisam mesajul NO
        }
    }

    // https://infoarena.ro/problema/ghizi
    void Ghizi(){
        ifstream fin("ghizi.in");
        ofstream fout("ghizi.out");

        int n, k, s = 101, t = 100;
        vector<pair<int, int>> vol;
        fin >> n >> k;
        Graph Graf(s);
        for(int i = 1; i <= n; ++i){
            int x, y;
            fin >> x >> y;
            vol.push_back({x, y});
            if(!Graf.verifyEdge(x, y)){     // Trasam muchii intre capetele intervalelor de timp la care este disponibil voluntarul 
                Graf.add(x, y, 1);
            }
            else{
                Graf.setCapacity(x, y, 1);      // Daca exista doi voluntati cu acelasi interval de disponibilitate, incrementam capacitatea muchiei
            }
        }
        Graf.add(s, 0, k);      // Se creeaza un nod de start s, cu muchie de capacitate k catre nodul 0
        Graf.maxFlow(s, t);     // Facem flux maxim pe graf
        vector<int> res;
        for(int i = 0; i < vol.size(); ++i){
            if(Graf.getFlow()[vol[i].first][vol[i].second]){        // Daca muchia este saturata, introducem voluntarul i
                res.push_back(i + 1);
                Graf.setFlow(vol[i].first, vol[i].second, -1);      // Scadem fluxul pentru a impiedica introducerea aceluiasi voluntar
            }
        }
        fout << res.size() << "\n";
        for(auto it : res){
            fout << it << " ";
        }
    }

    // https://www.infoarena.ro/problema/senat
    void Senat(){
        ifstream fin("senat.in");
        ofstream fout("senat.out");

        int n, m;
        fin >> n;
        fin >> m;
        fin.get();
        char a[10000];
        int s = 0, t = n + m + 1;
        Graph Graf(t);
        for(int i = 1; i <= m; ++i){
            fin.getline(a, 10000);      // Citim membrii comisiei i
            int nr = 0, size = strlen(a);
            for(int j = 0; j < size; ++j){
                if(isdigit(a[j])){
                    nr = nr * 10 + (a[j] - '0');
                }
                else if(a[j] == ' ' && j < size - 1){
                    Graf.add(nr, i + n, 1);     // Trasam muchie de capacitate 1 intre senatorul de index nr si comisia i
                    nr = 0;
                }
            }
            Graf.add(nr, i + n, 1);
            Graf.add(i + n, t, 1);      // Se creeaza un nod de final t cu muchie de capacitate 1 catre fiecare comisie
        }
        for(int i = 1; i <= n; ++i){
            Graf.add(s, i, 1);      // Se creeaza un nod imaginar de start s cu muchie de capacitate 1 catre fiecare senator
        }
        int total = Graf.maxFlow(s, t);     // Facem flux maxim pe graf
        if(total == m){     // Daca fluxul maxim este egal cu numarul de comisii, avem solutie
            for(int i = 1; i <= m; ++i){
                for(int j = 1; j <= n; ++j){
                    if(Graf.getFlow()[j][i + n] == 1){      // Daca fluxul pe muchia dintre senator si comisie este 1, muchia este saturata
                        fout << j << "\n";
                        break;
                    }
                }
            }
        }
        else{
            fout << 0;
        }
    }

    // https://www.infoarena.ro/problema/paznici
    void Paznici(){
        ifstream fin("paznici.in");
        ofstream fout("paznici.out");

        int n, m;
        fin >> n >> m;
        int s = 0, t = n + m + 1;
        Graph Graf(t);
        for(int i = 1; i <= n; ++i){
            string a;
            fin >> a;
            for(int j = 0; j < a.size(); ++j){
                if(a[j] == '1'){
                    Graf.add(i, j + n + 1, 1);      // Daca matricea pe linia i si coloana j are valoarea 1, trasam o muchie de capacitate 1 intre linia i si coloana j
                }
            }
            Graf.add(s, i, 1);      // // Se creeaza un nod imaginar de start s cu muchie de capacitate 1 catre fiecare linie
        }
        for(int i = 1; i <= m; ++i){
            Graf.add(i + n, t, 1);      // // Se creeaza un nod de final t cu muchie de capacitate 1 catre fiecare coloana
        }
        Graf.maxFlow(s, t);     // Facem flux maxim pe graf
        Graf.findMinimumVertexCover(0);     // Marcam nodurile vizitate ce ne vor da un raspuns pentru minimum vertex cover
        string res = "";
        for(int i = 1; i <= n; ++i){
            if(!Graf.verifyIfVisited(i)){       // Luam nodurile nevizitate din prima coloana
                res += ('A' + i - 1);
            }
        }
        for(int i = 1; i <= m; ++i){
            if(Graf.verifyIfVisited(i + n)){    // Luam nodurile vizitate din a doua coloana
                res += ('a' + i - 1);
            }
        }
        fout << res;
    }

    // https://csacademy.com/contest/archive/task/no-prime-sum/
    void NoPrimeSum(){
        int n;
        vector<int> numbers, leftNodes, rightNodes;
        vector<bool> primes(200005, true);
        cin >> n;
        for(int i = 1; i <= n; ++i){
            int x;
            cin >> x;
            numbers.push_back(x);
        }
        for(int i : numbers){
            if(i % 2 == 0){
                leftNodes.push_back(i);     // Pe prima coloana de noduri se vor plasa numerele pare 
            }
            else{
                rightNodes.push_back(i);    // Pe cea de a doua coloana se vor plasa numerele impare
            }
        }
        primes[0] = primes[1] = false;
        for(int i = 2; i <= 200005 / 2; ++i){       // Ciurul lui Eratostene
            if(primes[i]){
                for(int j = 2; j * i <= 200005; ++j){
                    primes[i * j] = false;
                }
            }
        }
        int s = 0, t = leftNodes.size() + rightNodes.size() + 1;    // Se creeaza doua noduri imaginare s si t
        Graph Graf(t);
        for(int i = 0; i < leftNodes.size(); ++i){
            for(int j = 0; j < rightNodes.size(); ++j){
                if(primes[leftNodes[i] + rightNodes[j]]){       // Daca suma unui numar din stanga cu unul din dreapta este un numar prim, trasam muchie de capacitate 1 intre ele
                    int u = i + 1, v = j + 1 + leftNodes.size();
                    Graf.add(u, v, 1);
                }
            }
        }
        for(int i = 1; i <= leftNodes.size(); ++i){
            Graf.add(s, i, 1);      // Trasam muchii de capacitate 1 de la nodul de start la nodurile de pe coloana din stanga
        }
        for(int i = 1; i <= rightNodes.size(); ++i){
            Graf.add(i + leftNodes.size(), t, 1);   // Trasam muchii de capacitate 1 de la nodul de final la nodurile de pe coloana din dreapta
        }
        Graf.maxFlow(s, t);     // Facem flux maxim pe graf
        vector<int> sol;
        Graf.findMinimumVertexCover(s);     // Cautam nodurile pentru minimum vertex cover
        for(int i = 0; i < leftNodes.size(); ++i){
            if(!Graf.verifyIfVisited(i + 1)){
                sol.push_back(leftNodes[i]);
            }
        }
        for(int i = 0; i < rightNodes.size(); ++i){
            if(Graf.verifyIfVisited(i + leftNodes.size() + 1)){
                sol.push_back(rightNodes[i]);
            }
        }
        cout << sol.size() << "\n";
        for(auto it : sol){
            cout << it << " ";
        }
    }

    // https://leetcode.com/problems/reconstruct-itinerary/description/
    vector<string> findItinerary(vector<vector<string>>& tickets){
        unordered_map<string, vector<string>> Graph;
        for(auto ticket : tickets){
            Graph[ticket[0]].push_back(ticket[1]);      // Trasam o muchie de la orasul din ticket[0] la orasul din ticket[1]
        }
        for(auto& [airport, dest] : Graph){
            sort(dest.rbegin(), dest.rend());   // Sortam vecinii fiecarui oras, deoarece dorim sa obtinem un ciclu Eulerian minim lexicografic
        }
        vector<string> sol;
        findItineraryHelper("JFK", sol, Graph);     // Apelam helper-ul pentru problema
        reverse(sol.begin(), sol.end());    // Cum helper-ul foloseste un algoritm recursiv, solutia trebuie inversata
        return sol;
    }

    // https://leetcode.com/problems/valid-arrangement-of-pairs/
    vector<vector<int>> validArrangement(vector<vector<int>>& pairs){
        int n = pairs.size();
        unordered_map<int, vector<int>> Graph;
        unordered_map<int, int> inDeg, outDeg;
        for(auto pair : pairs){     // Crestem gradul interior si exterior pentru fiecare nod si trasam muchii intre capetele fiecarei perechi
            inDeg[pair[1]]++;
            outDeg[pair[0]]++;
            Graph[pair[0]].push_back(pair[1]);
        }
        int start = -1;
        for(auto pair : Graph){     // Inceputul lantului va avea gradul interior mai mic cu 1 decat gradul exterior
            int i = pair.first;
            if(outDeg[i] == inDeg[i] + 1){
                start = i;
            }
        }
        if(start == -1){    // Daca niciun nod nu are gradul interior mai mic cu 1 decat gradul exterior, se pleaca de la primul nod din graf
            start = (*Graph.begin()).first;
        }
        vector<vector<int>> sol;
        validArrangementHelper(start, sol, Graph);      // Se apeleaza helper-ul pentru problema
        reverse(sol.begin(), sol.end());     // Cum helper-ul foloseste un algoritm recursiv, solutia trebuie inversata
        return sol;
    }

    // https://leetcode.com/problems/shortest-path-visiting-all-nodes/
    int shortestPathLength(vector<vector<int>>& graph){
        int n = graph.size();
        int allVisitedConfig = (1 << n) - 1;    // Construim o masca de biti cu toate nodurile vizitate (daca bitul i este 1, nodul i a fost vizitat)
        vector<vector<bool>> vis(allVisitedConfig + 1, vector<bool>(n + 1, false));
        queue<pair<int, pair<int, int>>> q;     // {currConfig, {node, dist}}
        for(int i = 0; i < n; ++i){     // Introducem in coada toate nodurile cu masca lor de biti respectiva (doar bitul i, corespunzator nodului i, este 1)
            q.push({(1 << i), {i, 0}});
            vis[1 << i][i] = true;
        }
        while(!q.empty()){      // Facem un algoritm BFS care gaseste drumul de distanta minima care contine toate nodurile
            int currConfig = q.front().first;       // Nodurile vizitate pana acum
            int node = q.front().second.first;      // Nodul la care ne aflam acum
            int dist = q.front().second.second;     // Distanta parcursa pana acum
            q.pop();
            if(currConfig == allVisitedConfig){     // Daca toate nodurile au fost vizitate, returnam distanta
                return dist;
            }
            for(auto neigh : graph[node]){
                int newConfig = currConfig | (1 << neigh);      // Construim noua masca, marcand bitul vecinului cu 1 
                if(!vis[newConfig][neigh]){     // Daca nu am vizitat vecinul i cu configuratia noua, il introducem in coada si crestem distanta
                    q.push({newConfig, {neigh, dist + 1}});
                    vis[newConfig][neigh] = true;   // Marcam nodul cu noua configuratie ca vizitate
                }
            }
        }
        return -1;
    }

    // https://infoarena.ro/problema/negot
    void Negot(){
        ifstream fin("negot.in");
        ofstream fout("negot.out"); 

        int n, m, k;
        fin >> n >> m >> k;
        int s = 0, t = m + n + 1;
        Graph Graf(t);
        for(int i = 1; i <= n; ++i){
            int x;
            fin >> x;
            for(int j = 1; j <= x; ++j){
                int y;
                fin >> y;
                Graf.add(i, n + y, 1);      // Trasam muchie de capacitate 1 de la producatorul i la magazinul y
            }
            Graf.add(s, i, k);      // Trasam muchie de capacitate k de la nodul de start s la toti cei n producatori
        }
        for(int i = 1; i <= m; ++i){;
            Graf.add(n + i, t, 1);      // Trasam muchie de capacitate 1 de la nodul de final t la toate cele m magazine
        }
        fout << Graf.maxFlow(s, t);     // Raspunsul este fluxul maxim pe graf
    }
};

int main(){
    Solution s;
    s.Negot();
    return 0;
}