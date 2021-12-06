// C++ program for Dijkstra's shortest path algorithm using adjacency list
// representation of graph
// Main program

#include "Dijkstra_on_d-ary.h"

void main() {
  // Create the graph 
  int V = 1, n = 10 * 10 * 10 + 1;
  Graph* graph;
  srand(static_cast<unsigned int>(time(0)));

  for (V; V < n; V += 100) {
    graph = createGraph(V);

    for (int i = 1; i < V; i++) {
      for (int j = i; j < V; j += rand() % 5) 
        addEdge(graph, i - 1, j, rand());
    }

    dijkstra(graph, rand()%V);

    printf("Here is your dist\n");
    //system("pause");
  }

  //addEdge(graph, 0, 1, 4);
  //addEdge(graph, 0, 2, 2);
  //addEdge(graph, 0, 3, 5);
  //addEdge(graph, 1, 2, 1);
  //addEdge(graph, 1, 3, 4);
  //addEdge(graph, 2, 3, 6);


  //addEdge(graph, 0, 1, 4);
  //addEdge(graph, 0, 7, 8);
  //addEdge(graph, 1, 2, 8);
  //addEdge(graph, 1, 7, 11);
  //addEdge(graph, 2, 3, 7);
  //addEdge(graph, 2, 8, 2);
  //addEdge(graph, 2, 5, 4);
  //addEdge(graph, 3, 4, 9);
  //addEdge(graph, 3, 5, 14);
  //addEdge(graph, 4, 5, 10);
  //addEdge(graph, 5, 6, 2);
  //addEdge(graph, 6, 7, 1);
  //addEdge(graph, 6, 8, 6);
  //addEdge(graph, 7, 8, 7);
}
