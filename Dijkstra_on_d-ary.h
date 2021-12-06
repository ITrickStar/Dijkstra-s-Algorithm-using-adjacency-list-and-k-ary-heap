// C++ program for Dijkstra's shortest path algorithm using adjacency list
// representation of graph 
// Utility program

#pragma once
#include <iostream>

// Sets the d-ary of minHeap Heaparr
const int k = 15;

// Node in adjacency list
struct AdjListNode {
  int dest;
  int weight;
  AdjListNode* next;
};

// Adjacency list
class AdjList {
 public:
  // Pointer to head node of list
  AdjListNode* head;

  AdjList() : head(NULL) {}
  ~AdjList() 
  {
    while (head != NULL) {
      AdjListNode* elem;
      elem = head;
      head = head->next;
      delete elem;
    }
  }
};

// Function to create a new adjacency list node
AdjListNode* newAdjListNode(int dest, int weight) {
  struct AdjListNode* newNode =
      (struct AdjListNode*)malloc(sizeof(struct AdjListNode));
  newNode->dest = dest;
  newNode->weight = weight;
  newNode->next = NULL;
  return newNode;
}

// Graph is an array of adjacency lists.
// Size of array will be V (number of vertices in graph)
class Graph {
 public:
  int V;
  AdjList* arr;

  Graph() : V(0), arr(NULL) {}
  ~Graph() { delete[] arr; }
};

// Function that creates a graph of V vertices
Graph* createGraph(int V) {
  Graph* graph = (Graph*)malloc(sizeof(Graph));
  graph->V = V;

  graph->arr = (AdjList*)malloc(V * sizeof(AdjList));

  // Initialize each adjacency list
  // as empty by making head as NULL
  for (int i = 0; i < V; ++i) graph->arr[i].head = NULL;

  return graph;
}

// Adds an edge to an undirected graph
void addEdge(Graph* graph, int src, int dest, int weight) {
  // Add an edge from src to dest.
  // The node is added at the beginning
  AdjListNode* newNode = newAdjListNode(dest, weight);
  newNode->next = graph->arr[src].head;
  graph->arr[src].head = newNode;

  // Since graph is undirected,
  // add an edge from dest to src
  newNode = newAdjListNode(src, weight);
  newNode->next = graph->arr[dest].head;
  graph->arr[dest].head = newNode;
}

// Structure of a min heap node
struct MinHeapNode {
  int v;
  int dist;  // dist - distance between vertexes
};

// Class min heap
class MinHeap {
 public:
  // Number of heap nodes
  int size;
  // Capacity of minHeap
  int capacity;
  // This is for decreaseKey()
  int* pos;
  MinHeapNode** heaparr;

  MinHeap() : size(0), capacity(0), pos(NULL), heaparr(NULL) {}
  ~MinHeap() {
    delete[] pos;
    for (int i = 0; i < capacity; i++) delete[] heaparr[i];
    delete[] heaparr; 
  }
};

// Function to create a new MinHeapNode
MinHeapNode* newMinHeapNode(int v, int dist) {
  MinHeapNode* minHeapNode =
      (MinHeapNode*)malloc(sizeof(MinHeapNode));
  minHeapNode->v = v;
  minHeapNode->dist = dist;
  return minHeapNode;
}

// Function to create a minHeap
MinHeap* createMinHeap(int capacity) {
  MinHeap* minHeap = (MinHeap*)malloc(sizeof(MinHeap));
  minHeap->pos = (int*)malloc(capacity * sizeof(int));
  minHeap->size = 0;
  minHeap->capacity = capacity;
  minHeap->heaparr =
      (MinHeapNode**)malloc(capacity * sizeof(MinHeapNode*));
  return minHeap;
}

// Function to swap two nodes of minHeap.
// Needed for heapify
void swapMinHeapNode(MinHeapNode** a, MinHeapNode** b) {
  MinHeapNode* t = *a;
  *a = *b;
  *b = t;
}

// Function to heapify at given idx
// This function also updates position of nodes when they are swapped.
void minHeapify(MinHeap* minHeap, int idx) {
  int smallest;
  smallest = idx;

  for (int i = 1; i <= k; i++)
    if (k * idx + i < minHeap->size &&
        minHeap->heaparr[k * idx + i]->dist < minHeap->heaparr[smallest]->dist)
      smallest = k * idx + i;

  if (smallest != idx) {
    // The nodes to be swapped in min heap
    MinHeapNode* smallestNode = minHeap->heaparr[smallest];
    MinHeapNode* idxNode = minHeap->heaparr[idx];

    // Swap positions
    minHeap->pos[smallestNode->v] = idx;
    minHeap->pos[idxNode->v] = smallest;

    // Swap nodes
    swapMinHeapNode(&minHeap->heaparr[smallest], &minHeap->heaparr[idx]);

    minHeapify(minHeap, smallest);
  }
}

// Checks given minHeap is empty or not
int isEmpty(MinHeap* minHeap) { return minHeap->size == 0; }

// Function to extract minimum node from heap
MinHeapNode* extractMin(MinHeap* minHeap) {
  if (isEmpty(minHeap)) return NULL;

  // Store the root node
  MinHeapNode* root = minHeap->heaparr[0];

  // Replace root node with last node
  MinHeapNode* lastNode = minHeap->heaparr[minHeap->size - 1];
  minHeap->heaparr[0] = lastNode;

  // Update position of last node
  minHeap->pos[root->v] = minHeap->size - 1;
  minHeap->pos[lastNode->v] = 0;

  // Reduce heap size and heapify root
  --minHeap->size;
  minHeapify(minHeap, 0);

  return root;
}

// Function to decrease dist value of a given vertex v.
// This function uses pos[] of min heap to get the current index of node in min heap

void decreaseKey(MinHeap* minHeap, int v, int dist) {
  // Get the index of v in  heap array
  int i = minHeap->pos[v];

  // Get the node and update its dist value
  minHeap->heaparr[i]->dist = dist;

  // If tree is not heapified.
  while (i && minHeap->heaparr[i]->dist < minHeap->heaparr[(i - 1) / k]->dist) {
    // Swap this node with its parent
    minHeap->pos[minHeap->heaparr[i]->v] = (i - 1) / k;
    minHeap->pos[minHeap->heaparr[(i - 1) / k]->v] = i;
    swapMinHeapNode(&minHeap->heaparr[i],
                    &minHeap->heaparr[(i - 1) / k]);  // change it

    // move to parent index
    i = (i - 1) / k;
  }
}

// Function to check if a given vertex(v) is in min heap or not
bool isInMinHeap(MinHeap* minHeap, int v) {
  if (minHeap->pos[v] < minHeap->size) return true;
  return false;
}

// Function used to print the solution
void printDist(int dist[], int n) {
  printf("Vertex   Distance from Source\n");
  for (int i = 0; i < n; ++i) printf("%d \t\t %d\n", i, dist[i]);
}

// The main function that calculates
// distances of shortest paths from src to all vertices.
void dijkstra(Graph* graph, int src) {
  // Get the number of vertices in graph
  int V = graph->V;

  // dist - shortest distances we are looking for minimum weight edge in cut
  int* dist = new int[V];

  // minHeap represents set of weights
  MinHeap* minHeap = createMinHeap(V);
  // Initially size of min heap is equal to V
  minHeap->size = V;

  // Initialize minHeap with all vertices
  // dist value of all vertices is INT_MAX
  for (int v = 0; v < V; ++v) {
    dist[v] = INT_MAX;
    minHeap->heaparr[v] = newMinHeapNode(v, dist[v]);
    minHeap->pos[v] = v;
  }

  // Make dist value of src vertex as 0 to extract it first
  minHeap->heaparr[src] = newMinHeapNode(src, dist[src]);
  minHeap->pos[src] = src;
  dist[src] = 0;
  decreaseKey(minHeap, src, dist[src]);


  // minHeap contains all nodes whose shortest distance is not yet finalized
  while (!isEmpty(minHeap)) {
    // Extract the vertex with
    // minimum distance value

    MinHeapNode* minHeapNode = extractMin(minHeap);

    // Store the extracted vertex number
    int u = minHeapNode->v;

    // Traverse through all adjacent
    // Vertices and update distance values of u
    AdjListNode* pCrawl = graph->arr[u].head;
    while (pCrawl != NULL) {
      int v = pCrawl->dest;

      // Checking if shortest distance to v is less than it's previously calculated distance
      if (isInMinHeap(minHeap, v) && dist[u] != INT_MAX &&
          pCrawl->weight + dist[u] < dist[v]) {
        dist[v] = dist[u] + pCrawl->weight;

        // update distance
        decreaseKey(minHeap, v, dist[v]);
      }
      pCrawl = pCrawl->next;
    }
  }

  // Print the calculated shortest distances
  printDist(dist, V);

  delete[] dist;
}
