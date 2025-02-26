#include "graph.hpp"
#include "logger.hpp"

/*
* @brief Class to find path between two nodes in a graph
*/
class Solution {

public:
    Graph graph;
    
    /*
    * @brief Check if path exists between source and destination
    * @param n: number of nodes
    * @param edges: vector of edges
    * @param source: source node
    * @param destination: destination node
    * @param alg: algorithm to find path (1. BFS, 2. DFS)
    * @return true if path exists, false otherwise
    */
    bool validPath(int n, std::vector<std::vector<int>>& edges, int source, int destination, int alg = 1) 
    {
        
        bool graph_created = this->graph.create_graph(edges);
        Path path_data = this->graph.does_path_exist(source, destination, alg);
        
        if (path_data.flag) 
        {
            std::cout << "Path exists: ";
            for (int node : path_data.path) {
                std::cout << node << " ";
            }
            std::cout << std::endl;
        } 
        else 
        {
            std::cout << "No path exists" << std::endl;
        }

        return path_data.flag;
    }
};


/*
* @brief Get test case and method from command line
* @param None
* @return vector of test case and method
*/
std::vector<int> get_data_from_cmd(logger log)
{
    std::vector<int> data;
    int test_case = 1;
    int method = 1;

    log.info("Please select test case (1-5): ", false);
    std::cin >> test_case;
    data.push_back(test_case);

    log.info("Please select method (1. BFS, 2. DFS, 3. Dijkstra 4. A-star): ", false);
    std::cin >> method;
    data.push_back(method);

    return data;
}

int main() {
    Solution solution;
    logger log;

    log.info("Path Finder");
    log.info("--------------------");

    std::vector<int> data = get_data_from_cmd(log);
    int test_case = data[0];
    int method = data[1];

    if (test_case < 1 || test_case > 5) {
        log.error("Invalid test case");
        return 0;
    }
    if (method < 1 || method > 4) {
        log.error("Invalid method");
        return 0;
    }

    switch (test_case)
    {
        case 1:
        {
            // Test case 1: Simple path exists
            std::vector<std::vector<int>> edges1 = {{0, 1}, {1, 2}, {2, 3}};
            log.info("Test Case 1:");
            solution.validPath(4, edges1, 0, 3, method);  // Expected: Path exists: 0 1 2 3
        }
        break;

        case 2:
        {
            // Test case 2: No path exists
            std::vector<std::vector<int>> edges2 = {{0, 1}, {2, 3}};
            log.info("Test Case 2:");
            solution.validPath(4, edges2, 0, 3, method);  // Expected: No path exists
        }
        break;

        case 3:
        {
            // Test case 3: Direct edge between source and destination
            std::vector<std::vector<int>> edges3 = {{0, 1}};
            log.info("Test Case 3:");
            solution.validPath(2, edges3, 0, 1, method);  // Expected: Path exists: 0 1
        }
        break;


        case 4:
        {
            // Test case 4: Graph with multiple paths (multiple parents for nodes)
            std::vector<std::vector<int>> edges4 = {{0, 1}, {0, 2}, {1, 3}, {2, 3}};
            log.info("Test Case 4:");
            solution.validPath(4, edges4, 0, 3, method);  // Expected: Path exists: 0 1 3 or 0 2 3
        }
        break;

        case 5:
        {
            // Test case 5: Path with backtracking
            std::vector<std::vector<int>> edges5 = {{0, 1}, {1, 2}, {2, 3}, {3, 4}};
            log.info("Test Case 5:");
            solution.validPath(5, edges5, 0, 4, method);  // Expected: Path exists: 0 1 2 3 4
        }
        break;

        default:
            log.warning("No test case executed");
            break;
    }

    return 0;
}
