#include <bits/stdc++.h>

using namespace std;

/////////////////////////////////////////////// CLASA GRAF //////////////////////////////////////////////
class Graph {
private:
    vector<vector<int>> adjList;
    bool directed;

public:
    Graph(int size, bool directed, const vector<vector<int>> &edges){
        this->directed = directed;
        this->adjList.resize(size + 1);
        for(auto edge : edges){
            this->adjList[edge[0]].push_back(edge[1]);
            if(this->directed == false){
                this->adjList[edge[1]].push_back(edge[0]);
            }
        }
    }
    Graph(const Graph& obj){    // Copy constructor
        this->adjList = obj.adjList;
        this->directed = obj.directed;
    }
    Graph& operator = (const Graph& obj){   // = operator overload
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
    vector<int> findTopologicalSort(){      // Returns a topological sort solution for a directed graph with NO cycles
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
        if(res.size() < n){
            res.clear();
        }
        return res;
    }
};
/////////////////////////////////////////////// CLASA GRAF //////////////////////////////////////////////


/////////////////////////////////////////////// CLASA SOLUTIE //////////////////////////////////////////////
class Solution {
public:
    bool possibleBipartition(int n, vector<vector<int>>& dislikes) {    // https://leetcode.com/problems/possible-bipartition/
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

    vector<int> findOrder(int numCourses, vector<vector<int>>& prerequisites) {     // https://leetcode.com/problems/course-schedule-ii/
        Graph G(numCourses, true, prerequisites);
        vector<int> res =  G.findTopologicalSort();
        reverse(res.begin(), res.end());
        return res;
    }
};
/////////////////////////////////////////////// CLASA SOLUTIE //////////////////////////////////////////////


int main(){
    
}