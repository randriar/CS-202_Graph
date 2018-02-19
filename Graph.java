import java.io.*;
import java.util.*;

/*
 * CS202 Graph Algorithms
 */
public class Graph {

    int n; // number of vertices
    int m; // number of edges
    boolean directed, weighted;
    Vertex[] V; // vertices
    int timeStamp;
    ArrayList<Vertex> topo;
    int componentId=0;
    ArrayList<Edge> edges;
    /**************************************************
     * The constructor, read the input graph from a file. 
     * The first line of the file contains int n, int m, boolean directed, boolean weighted. 
     * M line follow, each specifying an edge: 
     * int start, int end of the edge and, if weighted, double weight
     **************************************************/

    public Graph (String fileName) {      
        try (BufferedReader br = new BufferedReader(new FileReader(fileName))) {
            edges=new ArrayList<Edge>(m);
            // read a line and use the space symbol to split it into parts
            String[] firstLine = br.readLine().split(" ");

            // parse n, m, directed, weighted
            n = Integer.parseInt(firstLine[0]);
            m = Integer.parseInt(firstLine[1]);
            directed = firstLine[2].equals("directed");
            weighted = firstLine[3].equals("weighted");
            topo=new ArrayList<Vertex>(n);
            // create vertices 
            V = new Vertex[n];
            for (int i = 0; i < n; i++) {
                V[i] = new Vertex(i);
            }

            // create edges
            for (int i = 0; i < m; i++) {
                String[] line = br.readLine().split(" ");
                int start = Integer.parseInt(line[0]);
                int end = Integer.parseInt(line[1]);
                if (weighted) {
                    double weight = Double.parseDouble(line[2]);
                    V[start].E.add(new Edge(V[start], V[end], weight));
                    edges.add(new Edge(V[start], V[end], weight));
                    if (!directed) {
                        V[end].E.add(new Edge(V[end], V[start], weight));
                        edges.add(new Edge(V[end], V[start], weight));
                    }
                } else {
                    V[start].E.add(new Edge(V[start], V[end]));
                    edges.add(new Edge(V[start], V[end]));
                    if (!directed) {
                        V[end].E.add(new Edge(V[end], V[start]));
                        edges.add(new Edge(V[end], V[start]));
                    }
                }                
            }
        } catch (Exception e) {
            e.printStackTrace();
        } 
    }

    public Graph(int n){
        this.n=n;
        topo=new ArrayList<Vertex>(n);
        V=new Vertex[n];
        for(int i=0;i<n;i++){
            V[i] = new Vertex(i);
        }
    }

    /**************************************************
     * BFS
     **************************************************/

    public void bfs(Vertex s) {
        if (V.length == 0) return;

        for (Vertex v: V) {
            v.visited = false;
            v.dist = Integer.MAX_VALUE;
        }

        Queue<Vertex> q = new LinkedList<Vertex>();
        s.visited = true;
        s.dist = 0;
        q.add(s);

        while(!q.isEmpty()) {
            Vertex u  = q.poll();
            for (Edge e : u.E) {
                if (!e.end.visited) {
                    q.add(e.end);
                    e.end.visited = true;
                    e.end.dist = u.dist + 1;
                }
            }
        }
    }

    private void testBfs() {
        bfs(V[0]);
        System.out.println("Testing bfs: ");
        for (Vertex v: V) {
            if (v.visited) System.out.println(v.dist);
            else System.out.println("Infinity");
        }
    }

    /**************************************************
     * DFS, two versions: recursive and iterative
     **************************************************/

    public void dfs() {
        for (Vertex v: V) {
            v.visited = false;
        }

        timeStamp = 0;

        for (Vertex v: V) {
            if (!v.visited) explore(v);
        }
    }

    private void testDfs() {
        dfs();
        System.out.println("Testing dfs: ");
        for (Vertex v: V) {
            System.out.println(v.pre + ", " + v.post);            
        }
    }

    private void explore(Vertex v) {
        v.visited = true;
        preVisit(v);
        for (Edge e : v.E) {
            if (!e.end.visited) explore(e.end);
        }
        postVisit(v);
    }

    /** iterative dfs without recursion */
    public void iterativeDfs() {
        if (V.length == 0) return;
        for (Vertex v: V) {
            v.pre = v.post = -1;
        }

        timeStamp = 0;

        Stack<Vertex> s = new Stack<Vertex>();
        for (Vertex v: V) { // outer loop, 
            if (v.post > -1) continue; // u is done

            s.push(v);
            while(!s.isEmpty()) { // inner loop
                Vertex u = s.pop();
                u.component=componentId;
                if (u.post > -1) continue; // u is done
                else if (u.pre > -1) postVisit(u); // seen but not done
                else { // not seen
                    s.push(u); // put u back to stack, post will be set when it is poped again
                    preVisit(u);
                    for (Edge e: u.E) {
                        if (e.end.pre == -1) s.push(e.end);
                    }
                }
            }  
            componentId++;
        }
    }

    private void testIterativeDfs() {
        iterativeDfs();
        System.out.println("Testing iterativeDfs: ");
        for (Vertex v: V) {
            System.out.println(v.pre + ", " + v.post);            
        }
    }

    private void preVisit(Vertex v) {
        v.pre = timeStamp++;
    }

    private void postVisit(Vertex v) {
        v.post = timeStamp++;
        topo.add(v);
    }

    /**************************************************
     * Topological Sort
     **************************************************/
    public void topologicalSort(){
        iterativeDfs();
        Collections.reverse(topo);
    }

    private void testTopologicalSort(){
        topologicalSort();
        System.out.println("testing topological sort:");
        for(Vertex v:topo){
            System.out.println(v.id);
        }
    }

    /**************************************************
     * Strongly connected components
     **************************************************/
    public void scc(){
        Graph G2=reverse();
        G2.topologicalSort();
        Vertex []V2=new Vertex[n];
        int i=0;
        for(Vertex v:G2.topo){
            V2[i++]=V[v.id];
        }
        V=V2;
        componentId=0;
        /* Run DFS */
        iterativeDfs();
    }

    public Graph reverse(){
        Graph rev=new Graph(n);
        for(Vertex v:V){
            for(Edge e: v.E){
                int start=e.start.id;
                int end=e.end.id;
                rev.V[end].E.add(new Edge(rev.V[end],rev.V[start]));
            }
        }
        return rev;
    }

    private void testScc(){
        System.out.println("testing SCC:");
        scc();
        for(Vertex v:V){
            System.out.println(v.id+":"+v.component);
        }

    }

    /**************************************************
     * Dijkstra's shortest path
     **************************************************/
    public void dijkstra(Vertex s){
        System.out.println("The starting vertex is:"+s.id);
        if(V.length==0)return;
        for(Vertex v:V){
            v.dist=Double.POSITIVE_INFINITY;
            v.prev=null;
            v.visited=false;
        }
        s.dist=0;
        PriorityQueue<Vertex> q=new PriorityQueue();
        q.add(s);
        while(!q.isEmpty()){
            Vertex u=q.poll();
            if(u.visited) continue;
            u.visited=true;
            for(Edge e:u.E){
                if(e.end.dist>u.dist+e.weight){
                    e.end.dist=u.dist+e.weight;
                    e.end.prev=u;
                    q.add(e.end);
                }
            }
        }

    }

    private void testDijkstra(){
        System.out.println("testing Dijkstra:");
        dijkstra(V[0]);
        for(Vertex v:V){
            System.out.println(v.id+"~ "+v.dist);
        }

    }

    /**************************************************
     * Prim's spanning tree
     **************************************************/ 
    public void prim(Vertex s){
        System.out.println("The starting vertex is:"+s.id);
        if(V.length==0) return;
        for(Vertex v:V){
            v.cost=Double.POSITIVE_INFINITY;
            v.prev=null;
            v.visited=false;
        }
        s.dist=0;
        PriorityQueue<Vertex> q=new PriorityQueue();
        q.add(s);
        while(!q.isEmpty()){
            Vertex u=q.poll();
            if(u.visited) continue;
            u.visited=true;
            for(Edge e:u.E){
                if(e.end.cost>e.weight){
                    e.end.cost=e.weight;
                    e.end.prev=u;
                    q.add(e.end);
                }
            }
        }
    }

    private void testPrim(){
        System.out.println("Testing Prim's algorithm: ");
        prim(V[0]);
        for(Vertex v:V){
            System.out.println(v.id+" => "+v.cost);
        }
    }

    /**************************************************
     * Kruskal's spanning tree
     **************************************************/  
    public ArrayList<Edge> kruskal(){
        if(V.length==0)return null;
        //ArrayList<Edge> E=new ArrayList<Edge>();
        for(Vertex v:V){
            v.prev=null;
            v.rank=0;
            //E.addAll(v.E);
        }

        Collections.sort(edges,new Comparator<Edge>(){
                @Override
                public int compare(Edge e,Edge f){
                    return new Double(e.weight).compareTo(f.weight);
                }
            });
        ArrayList<Edge> mst=new ArrayList<Edge>(n-1);
        for(Edge e:edges){
            Vertex u=find(e.start);
            Vertex v=find(e.end);
            if(u!=v){
                mst.add(e);
                union(u,v);
            }
        }
        return mst;
    }

    /* find the parent of the set where v belongs to*/
    private Vertex find(Vertex v){
        while(v.prev!=null){
            v=v.prev;
        }
        return v;
    }

    /* unification of two sets*/
    private void union(Vertex v1,Vertex v2){
        if(v1.rank<v2.rank){
            v1.prev=v2;
        }
        if(v1.rank>v2.rank){
            v2.prev=v1;
        }else{
            v2.prev=v1;
            v1.rank++;
        }
    }

    private void testKruskal(){
        System.out.println("   Testing kruskal's alg:");
        for(Edge e:kruskal()){
            System.out.println(e.start.id+" "+e.end.id);
        }

    }

    /**************************************************
     * Bellman-Ford shortest path with negative edge weight
     **************************************************/ 
    public void bellmanFord(Vertex s){
        System.out.println("The starting vertex is:"+s.id);
        if(V.length==0)return;
        for(Vertex v:V){
            v.dist=Double.POSITIVE_INFINITY;
            v.prev=null;
            v.visited=false;
        }
        s.dist=0;
        for(int i=0;i<n-1;i++){
            for(Edge e:edges){
                update(e);
            }
        }
        for(Edge e:edges){
            if(update(e)){
                System.out.println("Error- found a negative cycle"); 
                return;
            }
        }
    }

    private boolean update(Edge e){
        if(e.end.dist>e.start.dist+e.weight){
            e.end.dist=e.start.dist+e.weight;
            e.end.prev=e.start;
            return true;
        }
        return false;
    }

    private void testBellmanFord(){
        System.out.println("Testing Bellman-Ford: ");
        bellmanFord(V[0]);
        for(Vertex v:V){
            System.out.println(v.id+" => "+v.dist);
        }
    }

    /**************************************************
     * Floyd-Warshall all-pairs shortest path
     **************************************************/  
    public double[][] floydWarshall(){
        double[][][] dist=new double[n][n][n];
        for(int i=0;i<n;i++){
            for(int j=0;j<n;j++){
                dist[i][j][0]=Double.POSITIVE_INFINITY;
            }
            for(Edge e:V[i].E){
                dist[i][e.end.id][0]=e.weight;
            }
        }
        for(int i=0;i<n;i++){
            for(int j=0;j<n;j++){
                for(int k=1;k<n;k++){
                    dist[i][j][k]=Math.min((dist[i][k][k-1]+dist[k][j][k-1]),dist[i][j][k-1]);
                }
            }
        }
        return dist[n];
    }

    private void testFloydWarshall(){
        double[][]dist=floydWarshall();
        System.out.println("Testing Floyd-Warshal: ");
        for(int i=0;i<n;i++){
            for(int j=0;j<n;j++){
                System.out.println(dist[i][j]);
            }
            System.out.println("");
        }
    }

    /**
     * REVIEW
     */
    public void dfs1(){
        for(Vertex v:V){
            v.visited=false;
        }
        timeStamp=0;
        for(Vertex v:V){
            if(!v.visited) explore1(v);
        }
    }

    private void explore1(Vertex v){
        v.visited=true;
        preVisit1(v);
        for(Edge e:v.E){
            if(!e.end.visited){
                explore(e.end);
            }
        }
        postVisit1(v);
    }

    private void preVisit1(Vertex v){
        v.pre=timeStamp++;
    }

    private void postVisit1(Vertex v){
        v.post=timeStamp++;
        topo.add(v);
    }

    private void testDfs1() {
        dfs1();
        System.out.println("Testing dfs: ");
        for (Vertex v: V) {
            System.out.println(v.pre + ", " + v.post);            
        }
    }

    public void iterativeDfs1(){
        if(V.length==0)return;
        for(Vertex v:V){
            v.pre=v.post=-1;
        }
        timeStamp=0;
        Stack<Vertex> s=new Stack<Vertex>();
        for(Vertex v:V){
            if(v.post>-1) continue;
            s.push(v);
            while(!s.isEmpty()){
                Vertex u=s.pop();
                u.component=componentId;
                if(u.post>-1) continue;
                else if(u.pre>-1) postVisit1(u);
                else{
                    s.push(u);
                    preVisit(u);
                    for(Edge e:u.E){
                        if(e.end.pre==-1) s.push(e.end);
                    }
                }
            }
            componentId++;
        }
    }

    private void testIterativeDfs1() {
        iterativeDfs1();
        System.out.println("Testing iterativeDfs: ");
        for (Vertex v: V) {
            System.out.println(v.pre + ", " + v.post);            
        }
    }

    private void topologicalSort1(){
        iterativeDfs1();
        Collections.reverse(topo);
    }

    private void testTopologicalSort1(){
        topologicalSort1();
        System.out.println("Testing topologicalSort: "+
            Arrays.toString(topo.toArray(new Vertex[topo.size()])));
    }

    public void scc1(){
        Graph g1=reverse1();
        g1.topologicalSort();
        Vertex[] ver=new Vertex[n];
        int i=0;
        for(Vertex v:g1.topo){
            ver[i++]=V[v.id];
        }
        V=ver;
        componentId=0;
        iterativeDfs1();
    }

    private Graph reverse1(){
        Graph g=new Graph(n);
        for(Vertex v:g.V){
            for(Edge e:v.E){
                g.V[e.end.id].E.add(new Edge(g.V[e.end.id],g.V[e.start.id]));
            }
        }
        return g;
    }

    private void testScc1(){
        scc1();
        System.out.println("Testing scc: ");
        for(Vertex v:V){
            System.out.println(v.id+":"+v.component);
        }
    }

    public void dijkstra1(Vertex s){
        System.out.println("The starting vertex is:"+s.id);
        if(V.length==0)return;
        for(Vertex v:V){
            v.dist=Double.POSITIVE_INFINITY;
            v.prev=null;
            v.visited=false;
        }
        PriorityQueue<Vertex> q=new PriorityQueue<Vertex>();
        s.dist=0;
        q.add(s);
        while(!q.isEmpty()){
            Vertex u=q.poll();
            if(u.visited) continue;
            u.visited=true;
            for(Edge e:u.E){
                if(e.end.dist>u.dist+e.weight){
                    e.end.dist=u.dist+e.weight;
                    e.end.prev=u;
                    q.add(e.end);
                } 
            }
        }
    }
    
    private void testDijkstra1(int i){
        System.out.println("Testing Dijkstra: ");
        dijkstra1(V[i]);
        for(Vertex v:V){
            System.out.println(v.id+" => "+v.dist);
        }
    }

    /**************************************************
     * Inner class Vertex
     **************************************************/

    class Vertex implements Comparable<Vertex>{
        int id;
        ArrayList<Edge> E;
        boolean visited;
        int pre, post; // time stamps
        double dist;
        Vertex prev;
        double weightedDist;
        int component;
        double cost;
        int rank;
        public Vertex(int i) {
            id = i;
            E = new ArrayList<Edge>();
        } 

        @Override
        public String toString(){
            return ""+id;
        }

        @Override
        public int compareTo(Vertex v2){
            if((this.dist-v2.dist)>0) return 1;
            else if ((this.dist-v2.dist)<0) return -1;
            else return 0;
        }
        /*
        @Override
        public int compare(Vertex e,Vertex f){
        return new Double(e.dist).compareTo(f.dist);
        }*/
    }

    /**************************************************
     * Inner class Edge
     **************************************************/    

    class Edge implements Comparator<Edge>{
        Vertex start;
        Vertex end;
        double weight;

        /** weighted edge */
        public Edge(Vertex s, Vertex e, double w) {
            start = s;
            end = e;
            weight = w;
        } 

        /** unweighted edge */
        public Edge(Vertex s, Vertex e) {
            start = s;
            end = e;
            weight = 1;
        }

        @Override
        public int compare(Edge e,Edge f){
            return new Double(e.weight).compareTo(f.weight);
        }
    }    

    /**************************************************
     * Main method
     **************************************************/

    public static void main(String[] args) {
        Graph G = new Graph("graph.txt");

        //G.testBfs();
        //G.testIterativeDfs();
        //G.testDfs();
        //G.testTopologicalSort();
        //G.testScc1();
        //G.testDijkstra1(0);
        //G.testPrim();
        //G.testBellmanFord();
        //G.testKruskal();
        //G.testFloydWarshall();
    }

}