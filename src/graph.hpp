#include <vector>
#include <queue>
#include <unordered_map>
#include <iostream>
#include <algorithm>

class Node
{
public:
    int name = 0;
    int x = 0;
    int y = 0;
    std::vector<int> neighbors;

    Node() {}

    Node(int name, int x, int y)
    {
        this->name = name;
        this->x = x;
        this->y = y;
    }
};

struct Path
{
    bool flag = false;
    std::vector<int> path = {};
};

class Graph_base
{
public:
    virtual bool create_graph(std::vector<std::vector<int>> edges)
    {
        std::cout << "No implementation to create graph" << std::endl;
        return false;
    }
    virtual void add_edge(std::vector<int> edge)
    {
        std::cout << "No implementation to add edge to graph" << std::endl;
    }
    virtual Path does_path_exist(int source, int destination, int alg)
    {
        std::cout << "No implementation to check if path exists" << std::endl;
        Path path_data;
        return path_data;
    }
    virtual Path bfs(int source, int destination)
    {
        std::cout << "BFS not implemented" << std::endl;
        Path path_data;
        return path_data;
    }
    virtual Path dfs(int source, int destination)
    {
        std::cout << "DFS not implemented" << std::endl;
        Path path_data;
        return path_data;
    }

};

class Graph : public Graph_base
{
public:
    std::unordered_map<int, Node> nodes;

    /*
    * @brief Create graph from edges
    * @param vector of edges
    * @return true if graph is created successfully
    *        false if graph creation fails
    */
    bool create_graph(std::vector<std::vector<int>> edges)
    {
        for (const auto &edge : edges)
        {
            add_edge(edge);
        }
        return true;
    }

    /*
    * @brief Add edge to graph
    * @param edge: vector of two nodes
    * @return None
    */
    void add_edge(std::vector<int> edge)
    {
        Node temp_node_1(edge[0], 0, 0);
        Node temp_node_2(edge[1], 0, 0);

        if (nodes.find(edge[0]) == nodes.end())
        {
            temp_node_1.neighbors.push_back(edge[1]);
            nodes[edge[0]] = temp_node_1;
        }
        else
        {
            nodes[edge[0]].neighbors.push_back(edge[1]);
        }

        if (nodes.find(edge[1]) == nodes.end())
        {
            temp_node_2.neighbors.push_back(edge[0]);
            nodes[edge[1]] = temp_node_2;
        }
        else
        {
            nodes[edge[1]].neighbors.push_back(edge[0]);
        }
    }

    /*
    * @brief Check if path exists between source and destination. Generates path if it exists. Uses BFS or DFS.
    * @param source: source node
    * @param destination: destination node
    * @param alg: algorithm to find path (1. BFS, 2. DFS)
    * @return Path data structure containing flag and path
    */
    Path does_path_exist(int source, int destination, int alg = 1)
    {
        Path path_data;
        if (source == destination)
        {
            path_data.flag = true;
            path_data.path = {source};
            return path_data;
        }

        if (alg == 1)
        {
            path_data = bfs(source, destination);
        }
        else if (alg == 2)
        {
            path_data = dfs(source, destination);
        }
        return path_data;
    }

private:
    /*
    * @brief Find path between source and destination using BFS
    * @param source: source node
    * @param destination: destination node
    * @return Path data structure containing flag and path
    */
    Path bfs(int source, int destination)
    {
        Path path_data;
        std::queue<int> node_queue;
        std::unordered_map<int, bool> visited;
        std::unordered_map<int, int> parent;

        node_queue.push(source);
        visited[source] = true;

        while (!node_queue.empty())
        {
            int current = node_queue.front();
            node_queue.pop();

            for (int neighbor : nodes[current].neighbors)
            {
                if (!visited[neighbor])
                {
                    visited[neighbor] = true;
                    parent[neighbor] = current;
                    node_queue.push(neighbor);

                    if (neighbor == destination)
                    {
                        path_data.flag = true;
                        reconstruct_path(source, destination, parent, path_data.path);
                        return path_data;
                    }
                }
            }
        }
        return path_data;
    }

    /*
    * @brief Find path between source and destination using DFS
    * @param source: source node
    * @param destination: destination node
    * @return Path data structure containing flag and path
    */
    Path dfs(int source, int destination)
    {
        Path path_data;
        std::unordered_map<int, bool> visited;
        std::vector<int> path;
        
        if (dfs_helper(source, destination, visited, path))
        {
            path_data.flag = true;
            path_data.path = path;
        }
        return path_data;
    }

    /*
    * @brief Helper function for DFS
    * @param current: current node
    * @param destination: destination node
    * @param visited: map to keep track of visited nodes
    * @param path: vector to store path
    * @return true if path exists, false otherwise
    */
    bool dfs_helper(int current, int destination, std::unordered_map<int, bool>& visited, std::vector<int>& path)
    {
        visited[current] = true;
        path.push_back(current);

        if (current == destination)
        {
            return true;
        }

        for (int neighbor : nodes[current].neighbors)
        {
            if (!visited[neighbor])
            {
                if (dfs_helper(neighbor, destination, visited, path))
                {
                    return true;
                }
            }
        }
        
        path.pop_back();
        return false;
    }

    /*
    * @brief Reconstruct path from parent map
    * @param source: source node
    * @param destination: destination node
    * @param parent: map containing parent of each node
    * @param path: vector to store path
    * @return None
    */
    void reconstruct_path(int source, int destination, std::unordered_map<int, int> &parent, std::vector<int> &path)
    {
        int current = destination;
        while (current != source)
        {
            path.push_back(current);
            current = parent[current];
        }
        path.push_back(source);
        std::reverse(path.begin(), path.end());
    }
};
