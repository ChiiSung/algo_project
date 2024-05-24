package main;
import javax.swing.*;
import java.awt.*;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.awt.geom.Point2D;
import java.util.ArrayList;
import java.util.Arrays;

public class GraphAlgorithmGUI extends JFrame {

    private JComboBox<String> algorithmComboBox;
    private JButton refreshNodesButton;
    private JButton runAlgorithmButton;
    private JPanel graphPanel;
    private JProgressBar progressBar;
    private int possiblePath;
    private int bruteForceStep;
    private int timeDelay = 0;

    private int[][] graph = new int[][]{
        // Locations:                      0     1      2     3     4     5     6     7     8     9     10    11    12    13    14    15    16    17    18    19    20    21    22
        /* 0 Batu Pahat */                {0,    7,		22,	  0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0},
        /* 1 Kampung Simpang Lima */      {7,	 0,     17,	  48,	0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0},
        /* 2 Parit Sulong */              {22,	 17,	0,    38,	26,   0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0},
        /* 3 Muar */                      {0,    48,	38,	  0,    0,    47,   0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0},
        /* 4 Pagoh */                     {0,    0,     26,   0,    0,    0,    75,   0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0},
        /* 5 Malacca */                   {0,    0,     0,    47,   0,    0,    38,   0,    57,	  0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0},
        /* 6 Simpang Ampat */             {0,    0,    	0,    0,    75,   38,   0,    43,	0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0},
        /* 7 Serembang */                 {0,    0,     0,    0,    0,    0,    43,	  0,    0,    15,   20,   27,   0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0},
        /* 8 Linggi */                    {0,    0,     0,    0,    0,    57,   0,    0,    0,    14,   18,   0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0},
        /* 9 Rantau */                    {0,    0,     0,    0,    0,    0,    0,    15,	14,   0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0},
        /* 10 Pedas */                    {0,    0,     0,    0,    0,    0,    0,    20,	18,   0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0},
        /* 11 USIM (Highway) */           {0,    0,     0,    0,    0,    0,    0,    27,	0,    0,    0,    0,    42,   27,   0,    0,    0,    0,    0,    0,    0,    0,    0},
        /* 12 Subang Jaya (Highway) */    {0,    0,     0,    0,    0,    0,    0,    0,    0,    0,    0,    42,   0,    19,   5,    0,    0,    0,    0,    0,    0,    0,    0},
        /* 13 Seri Kembangan */           {0,    0,     0,    0,    0,    0,    0,    0,    0,    0,    0,    27,   19,   0,    0,    0,    0,    0,    0,    0,    0,    0,    0},
        /* 14 Petaling Jaya (Highway) */  {0,    0,     0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    5,    0,    0,    32,   69,   0,    0,    0,    0,    0,    0},
        /* 15 Rawang */                   {0,    0,     0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    32,   0,    0,    53,   0,    0,    0,    0,    0},
        /* 16 Tanjung Karang */           {0,    0,     0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    69,   0,    0,    0,    82,   0,    0,    90,   0},
        /* 17 Tanjong Malim (Highway) */  {0,    0,     0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    53,   0,    0,    0,    0,    0,    0,    133},
        /* 18 Teluk Intan */              {0,    0,     0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    82,   0,    0,    44,   52,   0,    0},
        /* 19 Kampung Pahang */           {0,    0,     0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    44,   0,    0,    0,    62},
        /* 20 Bota Kanan */               {0,    0,     0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    52,   0,    0,    39,   43},
        /* 21 Kampung Sijagor */          {0,    0,     0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    90,   0,    0,    0,    39,   0,    0},
        /* 22 Ipoh */                     {0,    0,     0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    133,  0,    62,   43,   0,    0}
    };


    private String[] stations = {
    	    "Batu Pahat: 1.8548, 102.9325",
    	    "Kampung Simpang Lima: 1.8951, 102.6071",
    	    "Parit Sulong: 1.9947, 102.7890",
    	    "Muar: 2.0442, 102.4686",
    	    "Pagoh: 2.1941, 102.6713",
    	    "Malacca: 2.1896, 102.2501",
    	    "Simpang Ampat: 2.4653, 102.4246",
    	    "Serembang: 2.7297, 101.9381",
    	    "Linggi: 2.4981, 101.9457",
    	    "Rantau: 2.6102, 101.9041",
    	    "Pedas: 2.5791, 102.0087",
    	    "USIM (Highway): 2.8310, 101.7715",
    	    "Subang Jaya (Highway): 3.0032, 101.5807",
    	    "Kembangan: 2.6740, 101.6654",
    	    "Petaling Jaya (Highway): 3.1073, 101.6067",
    	    "Rawang: 3.3211, 101.5767",
    	    "Tanjung Karang: 3.4228, 101.1789",
    	    "Tanjong Malim (Highway): 3.6833, 101.5158",
    	    "Teluk Intan: 4.0122, 101.0211",
    	    "Kampung Pahang: 4.3104, 101.0906",
    	    "Bota Kanan: 4.3176, 100.8959",
    	    "Kampung Sijagor: 3.8811, 100.8621",
    	    "Ipoh: 4.5975, 101.0901"
    	};

    private Point2D.Double[] coordinates = new Point2D.Double[stations.length];

    private int[] shortestPath;
    private int shortestDistance;

    public GraphAlgorithmGUI() {
        setTitle("Graph Algorithm GUI");
        setSize(800, 600);
        setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
        setLocationRelativeTo(null);

        // Initialize components
        String[] algorithms = {"Brute Force", "Dijkstra's Algorithm", "Bellman-Ford Algorithm"};
        algorithmComboBox = new JComboBox<>(algorithms);
        refreshNodesButton = new JButton("Refresh Nodes");
        runAlgorithmButton = new JButton("Run Algorithm");
        progressBar = new JProgressBar();
        progressBar.setStringPainted(true);
        graphPanel = new JPanel() {
            @Override
            protected void paintComponent(Graphics g) {
                super.paintComponent(g);
                parseCoordinates();
                drawGraph(g);
                if (shortestPath != null) {
                    drawShortestPath(g);
                }
            }
        };

        // Set up layout
        JPanel controlPanel = new JPanel();
        controlPanel.add(new JLabel("Select Algorithm:"));
        controlPanel.add(algorithmComboBox);
        controlPanel.add(refreshNodesButton);
        controlPanel.add(runAlgorithmButton);
        controlPanel.add(progressBar);

        add(controlPanel, BorderLayout.NORTH);
        add(new JScrollPane(graphPanel), BorderLayout.CENTER);

        // Add action listeners
        refreshNodesButton.addActionListener(new ActionListener() {
            @Override
            public void actionPerformed(ActionEvent e) {
            	shortestPath = null;
            	shortestDistance = 0;
            	progressBar.setValue(0);
                graphPanel.repaint();
            }
        });

        runAlgorithmButton.addActionListener(new ActionListener() {
            @Override
            public void actionPerformed(ActionEvent e) {
            	shortestPath = null;
            	shortestDistance = 0;
                runSelectedAlgorithm();
            }
        });
    }

    private void parseCoordinates() {
        for (int i = 0; i < stations.length; i++) {
            String[] parts = stations[i].split(": ");
            String[] latLon = parts[1].split(", ");
            double latitude = Double.parseDouble(latLon[0]);
            double longitude = Double.parseDouble(latLon[1]);
            coordinates[i] = new Point2D.Double(latitude, longitude);
        }
    }private void drawGraph(Graphics g) {
        int nodeCount = graph.length;

        // Find the min and max latitude and longitude
        double minLat = Arrays.stream(coordinates).mapToDouble(p -> p.x).min().orElse(0);
        double maxLat = Arrays.stream(coordinates).mapToDouble(p -> p.x).max().orElse(0);
        double minLon = Arrays.stream(coordinates).mapToDouble(p -> p.y).min().orElse(0);
        double maxLon = Arrays.stream(coordinates).mapToDouble(p -> p.y).max().orElse(0);

        // Map coordinates to panel size with margin
        int width = graphPanel.getWidth() - 80;  // Adjust for the 10px margin on each side
        int height = graphPanel.getHeight() - 80;  // Adjust for the 10px margin on each side

        Point[] nodePositions = new Point[nodeCount];
        for (int i = 0; i < nodeCount; i++) {
            int x = (int) ((coordinates[i].y - minLon) / (maxLon - minLon) * width) + 40;  // Add margin
            int y = (int) ((maxLat - coordinates[i].x) / (maxLat - minLat) * height) + 40;  // Add margin and reflect Y-axis
            nodePositions[i] = new Point(x, y);
            g.fillOval(x - 5, y - 5, 10, 10);
            g.drawString(stations[i].split(": ")[0], x - 20, y - 10);
        }

        for (int i = 0; i < nodeCount; i++) {
            for (int j = 0; j < nodeCount; j++) {
                if (graph[i][j] != 0) {
                    g.drawLine(nodePositions[i].x, nodePositions[i].y, nodePositions[j].x, nodePositions[j].y);
                    int midX = (nodePositions[i].x + nodePositions[j].x) / 2;
                    int midY = (nodePositions[i].y + nodePositions[j].y) / 2;
                    g.drawString(Integer.toString(graph[i][j]), midX, midY);
                }
            }
        }
    }

    private void drawShortestPath(Graphics g) {
        if (shortestPath == null) return;

        Graphics2D g2d = (Graphics2D) g;
        Stroke defaultStroke = g2d.getStroke();

        // Set a thicker stroke for the shortest path
        g2d.setStroke(new BasicStroke(3));
        g.setColor(Color.RED);

        // Find the min and max latitude and longitude
        double minLat = Arrays.stream(coordinates).mapToDouble(p -> p.x).min().orElse(0);
        double maxLat = Arrays.stream(coordinates).mapToDouble(p -> p.x).max().orElse(0);
        double minLon = Arrays.stream(coordinates).mapToDouble(p -> p.y).min().orElse(0);
        double maxLon = Arrays.stream(coordinates).mapToDouble(p -> p.y).max().orElse(0);

        // Map coordinates to panel size with margin
        int width = graphPanel.getWidth() - 80;  // Adjust for the 10px margin on each side
        int height = graphPanel.getHeight() - 80;  // Adjust for the 10px margin on each side

        for (int i = 0; i < shortestPath.length - 1; i++) {
            int from = shortestPath[i];
            int to = shortestPath[i + 1];

            // Calculate adjusted coordinates for the shortest path lines
            int x1 = (int) ((coordinates[from].y - minLon) / (maxLon - minLon) * width) + 40;  // Add margin
            int y1 = (int) ((maxLat - coordinates[from].x) / (maxLat - minLat) * height) + 40;  // Add margin and reflect Y-axis
            int x2 = (int) ((coordinates[to].y - minLon) / (maxLon - minLon) * width) + 40;  // Add margin
            int y2 = (int) ((maxLat - coordinates[to].x) / (maxLat - minLat) * height) + 40;  // Add margin and reflect Y-axis

            g.drawLine(x1, y1, x2, y2);
        }

        // Reset the stroke and color to their default values
        g2d.setStroke(defaultStroke);
        g.setColor(Color.BLACK);
    }

    private void runSelectedAlgorithm() {
        String selectedAlgorithm = (String) algorithmComboBox.getSelectedItem();
        progressBar.setValue(0);
        switch (selectedAlgorithm) {
            case "Brute Force":
                runBruteForceAlgorithm();
                break;
            case "Dijkstra's Algorithm":
            	runDijkstraAlgorithm(0);
                break;
            case "Bellman-Ford Algorithm":
            	runBellmanFordAlgorithm(0);
                break;
        }
    }

    // Brute-force
    private void runBruteForceAlgorithm() {
    	possiblePath = 0;
    	countPossiblePaths(1, 0, new boolean[graph.length], graph.length - 1);
    	System.out.println(possiblePath);
        bruteForceStep = 0;

        shortestDistance = Integer.MAX_VALUE;
        progressBar.setIndeterminate(true);

        // Start time
        long startTime = System.currentTimeMillis();

        // Run the algorithm in a separate thread to avoid blocking the EDT
        SwingWorker<Void, Void> worker = new SwingWorker<Void, Void>() {
            @Override
            protected Void doInBackground() {
                bruteForceRecursive(0, new int[graph.length], 1, 0, new boolean[graph.length], graph.length - 1);
                return null;
            }

            @Override
            protected void done() {
                // End time
                long endTime = System.currentTimeMillis();
                long elapsedTime = endTime - startTime;

                progressBar.setValue(100);
                progressBar.setIndeterminate(false);
                graphPanel.repaint();
                displayMinPath(startTime);
            }
        };
        worker.execute();
    }

    private void countPossiblePaths(int pathNum, int layer, boolean[] visited, int destination) {
    	if (pathNum >= graph.length || layer == destination) {
            return ;
        }

        for (int i = 0; i < graph.length; i++) {
            if (graph[layer][i] != 0 && !visited[i]) {
                visited[i] = true;
                countPossiblePaths(pathNum + 1, i, visited, destination);
                visited[i] = false;
            }
            possiblePath++;
        }
    }

    public void bruteForceRecursive(int distance, int[] path, int pathNum, int layer, boolean[] visited, int destination) {
        if (layer == destination) {
            // Reached the end of the path or the destination is reached
            if (shortestDistance >= distance) {
                shortestDistance = distance;
                shortestPath = Arrays.copyOf(path, pathNum); // Update shortestPath with the current path
            }
            return;
        }	

        for (int i = 0; i < graph.length; i++) {
            if (graph[layer][i] != 0 && !visited[i]) {
                // If there's an edge from the current node to node i and i is not already visited
                visited[i] = true; // Mark i as visited
                path[pathNum] = i;
                graphPanel.repaint();
                // Update the distance by adding the distance from the current node to node i
                bruteForceRecursive(distance + graph[layer][i], path, pathNum + 1, i, visited, destination);
                visited[i] = false; // Backtrack: mark i as unvisited for other paths
            }
            try {
                Thread.sleep(timeDelay);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
            // Update progress bar
            bruteForceStep++;
            progressBar.setValue((int) ((double) bruteForceStep / (double) possiblePath * 100));
        }
    }
    
    //Dijkstra
    private void runDijkstraAlgorithm(int startNode) {
        progressBar.setIndeterminate(true);
        new Thread(new Runnable() {
            @Override
            public void run() {
            	// Start time
                long startTime = System.currentTimeMillis();
                
                int[] distances = new int[graph.length];
                int[] parentNodes = new int[graph.length];
                boolean[] visited = new boolean[graph.length];

                Arrays.fill(distances, Integer.MAX_VALUE);
                Arrays.fill(parentNodes, -1);

                distances[startNode] = 0;

                for (int i = 0; i < graph.length - 1; i++) {
                    int minDistanceNode = getMinDistanceNode(distances, visited);
                    visited[minDistanceNode] = true;

                    for (int j = 0; j < graph.length; j++) {
                        if (!visited[j] && graph[minDistanceNode][j] != 0 && distances[minDistanceNode] != Integer.MAX_VALUE &&
                                distances[minDistanceNode] + graph[minDistanceNode][j] < distances[j]) {
                            distances[j] = distances[minDistanceNode] + graph[minDistanceNode][j];
                            parentNodes[j] = minDistanceNode;
                            try {
								Thread.sleep(timeDelay);
							} catch (InterruptedException e) {
								// TODO Auto-generated catch block
								e.printStackTrace();
							}
                        }
                    }

                    // Update progress bar
                    progressBar.setValue((int) ((double) i / (double) (graph.length - 1) * 100));
                }

                // Reconstruct the shortest path
                shortestPath = new int[graph.length];
                int currentNode = graph.length - 1;
                int pathLength = 0;
                while (currentNode != -1) {
                    shortestPath[pathLength++] = currentNode;
                    currentNode = parentNodes[currentNode];
                    try {
						Thread.sleep(timeDelay);
					} catch (InterruptedException e) {
						// TODO Auto-generated catch block
						e.printStackTrace();
					}
                }

                // Reverse the path to get the correct order
                int[] reversedPath = new int[pathLength];
                for (int i = 0; i < pathLength; i++) {
                    reversedPath[i] = shortestPath[pathLength - i - 1];
                    shortestDistance += graph[reversedPath[i]][shortestPath[pathLength - i]];
                    try {
						Thread.sleep(timeDelay);
					} catch (InterruptedException e) {
						// TODO Auto-generated catch block
						e.printStackTrace();
					}
                }
                shortestPath = reversedPath;
                
                SwingUtilities.invokeLater(new Runnable() {
                    @Override
                    public void run() {
                    	progressBar.setValue(100);
                        progressBar.setIndeterminate(false);
                        graphPanel.repaint();
                        displayMinPath(startTime);
                    }
                });
            }
        }).start();
    }

    private int getMinDistanceNode(int[] distances, boolean[] visited) {
        int minDistance = Integer.MAX_VALUE;
        int minDistanceNode = -1;

        for (int i = 0; i < distances.length; i++) {
            if (!visited[i] && distances[i] <= minDistance) {
                minDistance = distances[i];
                minDistanceNode = i;
            }
            try {
                Thread.sleep(timeDelay);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
        return minDistanceNode;
    }

    //Bellman Ford Algorithm
    private void runBellmanFordAlgorithm(int source) {
	    progressBar.setIndeterminate(true);
	    new Thread(new Runnable() {
	        @Override
	        public void run() {
	        	// Start time
	            long startTime = System.currentTimeMillis();
	        	
	            int[] distances = new int[graph.length];
	            Arrays.fill(distances, Integer.MAX_VALUE);
	            distances[source] = 0;
	
	            int[] parentNodes = new int[graph.length];
	            Arrays.fill(parentNodes, -1);
	            
	            ArrayList<Edge> edges = getAllEdges(graph);
	
	            // Relax all edges |V| - 1 times
	            for (int i = 0; i < graph.length - 1; i++) {
	                for (Edge edge: edges) {
	                	int u = edge.getU();
	                	int v = edge.getV();
	                	int weight = edge.getWeight();
	                	if (distances[u] != Integer.MAX_VALUE && distances[u] + weight < distances[v]) {
	                        distances[v] = distances[u] + weight;
	                        parentNodes[v] = u;
	                    }
	                    try {
							Thread.sleep(timeDelay);
						} catch (InterruptedException e) {
							// TODO Auto-generated catch block
							e.printStackTrace();
						}
	                }
	                // Update progress bar
	                progressBar.setValue((int) ((double) i / (double) (graph.length - 1) * 100));
	            }
	
	            // Check for negative cycles
	            for (Edge edge : edges) {
	                int u = edge.getU();
	                int v = edge.getV();
	                int weight = edge.getWeight();
	
	                if (distances[u] != Integer.MAX_VALUE && distances[u] + weight < distances[v]) {
	                    progressBar.setIndeterminate(false);
	                    JOptionPane.showMessageDialog(GraphAlgorithmGUI.this, "Graph contains a negative cycle.");
	                    return;
	                }
	
	                try {
	                    Thread.sleep(timeDelay);
	                } catch (InterruptedException e) {
	                    e.printStackTrace();
	                }
	            }
	
	            // Reconstruct the shortest path
	            shortestPath = new int[graph.length];
	            int currentNode = graph.length - 1;
	            int pathLength = 0;
	            while (currentNode != -1) {
	                shortestPath[pathLength++] = currentNode;
	                currentNode = parentNodes[currentNode];
	                try {
						Thread.sleep(timeDelay);
					} catch (InterruptedException e) {
						// TODO Auto-generated catch block
						e.printStackTrace();
					}
	            }
	
	            // Reverse the path to get the correct order
	            int[] reversedPath = new int[pathLength];
	            for (int i = 0; i < pathLength; i++) {
	                reversedPath[i] = shortestPath[pathLength - i - 1];
	                shortestDistance += graph[reversedPath[i]][shortestPath[pathLength - i]];
	                try {
						Thread.sleep(timeDelay);
					} catch (InterruptedException e) {
						// TODO Auto-generated catch block
						e.printStackTrace();
					}
	            }
	            shortestPath = reversedPath;
	            
	            SwingUtilities.invokeLater(new Runnable() {
	                @Override
	                public void run() {
	                	progressBar.setValue(100);
	                    progressBar.setIndeterminate(false);
	                    graphPanel.repaint();
	                    displayMinPath(startTime);
	                }
	            });
	        }
	    }).start();
	}

    public void displayMinPath(long startTime) {
    	String[][] parts = new String[stations.length][2];
    	for(int i = 0; i < stations.length ; i++) {
            parts[i] = stations[i].split(": ");
    	}
    	String pathLocation = "";
    	for(int i = 0; i < shortestPath.length ; i++) {
    		pathLocation += parts[shortestPath[i]][0] + ", ";
    	}
        String pathString = pathLocation 
        		+ " with distance " + shortestDistance + "\n Time Used: " + (System.currentTimeMillis() - startTime) + "ms";
        JOptionPane.showMessageDialog(GraphAlgorithmGUI.this, "Shortest Path: " + pathString);
    }
    
    public static ArrayList<Edge> getAllEdges(int[][] graph) {
        ArrayList<Edge> edges = new ArrayList<>();
        for (int u = 0; u < graph.length; u++) {
            for (int v = 0; v < graph.length; v++) {
                if (graph[u][v] != 0) {
                    edges.add(new Edge(u, v, graph[u][v]));
                }
            }
        }
        return edges;
    }
    
    public static void main(String[] args) {
        SwingUtilities.invokeLater(new Runnable() {
            @Override
            public void run() {
                new GraphAlgorithmGUI().setVisible(true);
            }
        });
    }
}


class Edge {
    private int u;
    private int v;
    private int weight;

    public Edge(int u, int v, int weight) {
        this.u = u;
        this.v = v;
        this.weight = weight;
    }

    public int getU() {
        return u;
    }

    public int getV() {
        return v;
    }

    public int getWeight() {
        return weight;
    }
}
