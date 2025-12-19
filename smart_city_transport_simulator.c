#include<stdio.h>
#include<stdlib.h>
#include<string.h>

#define MAXN 100
#define MAXE 600
#define INF 1000000000
#define NONE_MODE 5



typedef struct{
    char name[32];
} City;

typedef struct{
    int id;
    int u, v;
    char mode[8];
    int time;
    int cost;
    int available;
} Edge;

typedef struct{
    City cities[MAXN];
    int cityCount;
    Edge edges[MAXE];
    int edgeCount;
    int adj[MAXN][MAXE];
    int deg[MAXN];
} Graph;

typedef struct{
    char preferredMode[8];
    int avoid[5];
    int switchPenalty;
} Settings;




Graph G;
Settings S;

int isDefaultCity[MAXN];

// time-based dijkstra
int distArr[MAXN][6];
int parentNodeArr[MAXN][6];
int parentModeArr[MAXN][6];
int parentEdgeArr[MAXN][6];

int lastPathNodes[500];
int lastPathEdges[500];
int lastPathLen=0;

// PQ (time)
int pqNode[5000];
int pqMode[5000];
int pqDist[5000];
int pqSz=0;

// PQ (cost)
int cpqNode[5000];
int cpqDist[5000];
int cpqSz=0;

// cost-based
int costDist[MAXN];
int costParentNode[MAXN];
int costParentEdge[MAXN];



int modeId(char*m){
    if(strcasecmp(m,"ROAD")==0) return 0;
    if(strcasecmp(m,"BUS")==0) return 1;
    if(strcasecmp(m,"METRO")==0) return 2;
    if(strcasecmp(m,"WALK")==0) return 3;
    if(strcasecmp(m,"AIR")==0) return 4;
    return -1;
}

char* modeName(int id){
    if(id==0) return "ROAD";
    if(id==1) return "BUS";
    if(id==2) return "METRO";
    if(id==3) return "WALK";
    if(id==4) return "AIR";
    if(id==5) return "NONE";
    return "?";
}

int modeBaseCost(int mid){
    if(mid==0) return 5;
    if(mid==1) return 10;
    if(mid==2) return 15;
    if(mid==3) return 0;
    if(mid==4) return 100;
    return 999;
}




void pqClear(){ pqSz=0; }

void pqPush(int node,int mode,int dist){
    int i=pqSz;
    pqNode[i]=node;
    pqMode[i]=mode;
    pqDist[i]=dist;
    pqSz++;

    while(i>0){
        int p=(i-1)/2;
        if(pqDist[p] <= pqDist[i]) break;

        int tn=pqNode[p], tm=pqMode[p], td=pqDist[p];
        pqNode[p]=pqNode[i]; pqMode[p]=pqMode[i]; pqDist[p]=pqDist[i];
        pqNode[i]=tn; pqMode[i]=tm; pqDist[i]=td;
        i=p;
    }
}

int pqPop(int *node,int *mode,int *dist){
    if(pqSz==0) return 0;

    *node = pqNode[0];
    *mode = pqMode[0];
    *dist = pqDist[0];

    pqSz--;
    pqNode[0]=pqNode[pqSz];
    pqMode[0]=pqMode[pqSz];
    pqDist[0]=pqDist[pqSz];

    int i=0;
    while(1){
        int l=2*i+1, r=2*i+2, b=i;
        if(l<pqSz && pqDist[l]<pqDist[b]) b=l;
        if(r<pqSz && pqDist[r]<pqDist[b]) b=r;
        if(b==i) break;

        int tn=pqNode[b], tm=pqMode[b], td=pqDist[b];
        pqNode[b]=pqNode[i]; pqMode[b]=pqMode[i]; pqDist[b]=pqDist[i];
        pqNode[i]=tn; pqMode[i]=tm; pqDist[i]=td;
        i=b;
    }
    return 1;
}




void cpqClear(){ cpqSz=0; }

void cpqPush(int node,int dist){
    int i=cpqSz;
    cpqNode[i]=node;
    cpqDist[i]=dist;
    cpqSz++;

    while(i>0){
        int p=(i-1)/2;
        if(cpqDist[p] <= cpqDist[i]) break;

        int tn=cpqNode[p], td=cpqDist[p];
        cpqNode[p]=cpqNode[i]; cpqDist[p]=cpqDist[i];
        cpqNode[i]=tn; cpqDist[i]=td;
        i=p;
    }
}

int cpqPop(int *node,int *dist){
    if(cpqSz==0) return 0;

    *node = cpqNode[0];
    *dist = cpqDist[0];

    cpqSz--;
    cpqNode[0]=cpqNode[cpqSz];
    cpqDist[0]=cpqDist[cpqSz];

    int i=0;
    while(1){
        int l=2*i+1, r=2*i+2, b=i;
        if(l<cpqSz && cpqDist[l]<cpqDist[b]) b=l;
        if(r<cpqSz && cpqDist[r]<cpqDist[b]) b=r;
        if(b==i) break;

        int tn=cpqNode[b], td=cpqDist[b];
        cpqNode[b]=cpqNode[i]; cpqDist[b]=cpqDist[i];
        cpqNode[i]=tn; cpqDist[i]=td;
        i=b;
    }
    return 1;
}




void initGraph(){
    int i,j;
    G.cityCount=0;
    G.edgeCount=0;

    for(i=0;i<MAXN;i++){
        isDefaultCity[i]=0;
        G.deg[i]=0;
        for(j=0;j<MAXE;j++) G.adj[i][j]=-1;
    }

    S.preferredMode[0]='\0';
    for(i=0;i<5;i++) S.avoid[i]=0;
    S.switchPenalty=3;

    lastPathLen=0;
}

int findCityIndex(char*name){
    int i;
    for(i=0;i<G.cityCount;i++){
        if(strcmp(G.cities[i].name,name)==0) return i;
    }
    return -1;
}

int createCity(char*name){
    if(G.cityCount>=MAXN) return -1;
    int idx = G.cityCount++;
    strcpy(G.cities[idx].name,name);
    return idx;
}

int getOrCreateCity(char*name){
    int i=findCityIndex(name);
    if(i!=-1) return i;
    return createCity(name);
}

void addEdgeInternal(int u,int v,char*mode,int time){
    if(u<0||v<0||u>=MAXN||v>=MAXN) return;
    if(G.edgeCount>=MAXE) return;

    int mid=modeId(mode);
    if(mid==-1) return;

    int e=G.edgeCount++;
    G.edges[e].id=e;
    G.edges[e].u=u;
    G.edges[e].v=v;
    strcpy(G.edges[e].mode,modeName(mid));
    G.edges[e].time=time;
    G.edges[e].cost=modeBaseCost(mid);
    G.edges[e].available=1;

    G.adj[u][G.deg[u]++]=e;
    G.adj[v][G.deg[v]++]=e;
}





 void buildDefaultCityGraph(){
    int A,B,C,D,E,F,Gc,H,I,J;
    A = getOrCreateCity("A");
    B = getOrCreateCity("B");
    C = getOrCreateCity("C");
    D = getOrCreateCity("D");
    E = getOrCreateCity("E");
    F = getOrCreateCity("F");
    Gc = getOrCreateCity("G");
    H = getOrCreateCity("H");
    I = getOrCreateCity("I");
    J = getOrCreateCity("J");

   
    // A–B WALK (ASCII shows only walk)
    addEdgeInternal(A,B,"WALK",4);

    // METRO ring (ASCII)
    addEdgeInternal(B,C,"METRO",7);
    addEdgeInternal(C,D,"METRO",6);
    addEdgeInternal(D,E,"METRO",9);
   // addEdgeInternal(E,A,"METRO",10);

   
   // addEdgeInternal(A,F,"ROAD",20);
    addEdgeInternal(B,Gc,"ROAD",18);
   // addEdgeInternal(C,H,"ROAD",22);
    addEdgeInternal(D,I,"ROAD",25);
    addEdgeInternal(E,J,"ROAD",19);

 
    addEdgeInternal(F,Gc,"ROAD",12);
    addEdgeInternal(Gc,H,"ROAD",14);
    addEdgeInternal(H,I,"ROAD",15);
    addEdgeInternal(I,J,"BUS",17);


    addEdgeInternal(F,B,"ROAD",12);
   // addEdgeInternal(H,D,"BUS",11);
    addEdgeInternal(I,E,"BUS",9);
   // addEdgeInternal(J,A,"BUS",13);
    addEdgeInternal(B,H,"BUS",13);


    addEdgeInternal(Gc,H,"WALK",6);
  //  addEdgeInternal(H,F,"WALK",7);


    addEdgeInternal(A,F,"AIR",40);
    addEdgeInternal(I,J,"BUS",30);

    // Mark all 10 cities as default
    for(int i=0;i<G.cityCount;i++) isDefaultCity[i]=1;
}






void printAsciiStatic(){
    printf("\n                 SMART CITY LAYOUT (STATIC ASCII VIEW)\n\n");

    printf("                            ( AIR )\n");
    printf("                      F -------- A\n");
    printf("                      |         / \\\n");
    printf("                      |        /   \\   (BUS to J)\n");
    printf("                 (ROAD)  (walk) /    (METRO)\n");
    printf("                      |      /       \\\n");
    printf("                      |   (BUS)      (METRO)\n");
    printf("                      \\---- B ==(METRO)== C ==(METRO)== D ==(METRO)== E\n");
    printf("                           |    //|       \\     //      \\\n");
    printf("                           |   // | (METRO)    //        \\\n");
    printf("                      H ---+  //   \\         //          +   \n  ");
    printf("                      (BUS)    \\     \\-------/         (BUS)\n");
    printf("                                 \\                    J   \n");
    printf("                                  \\________ I _________/\n");
    printf("                                       (AIR) (BUS)\n\n");
}





void printAddedCitiesSummary(){
    int anyAdded=0;

    for(int i=0;i<G.cityCount;i++){
        if(!isDefaultCity[i]){
            anyAdded=1;
            break;
        }
    }

    if(!anyAdded){
        printf("No additional cities added yet.\n");
        return;
    }

    printf("===================== ADDED CITIES =====================\n\n");

    for(int i=0;i<G.cityCount;i++){
        if(isDefaultCity[i]==0){
            printf("• %s\n", G.cities[i].name);

            int hasConn=0;
            for(int j=0;j<G.deg[i];j++){
                int ei = G.adj[i][j];
                if(ei<0) continue;

                Edge E = G.edges[ei];
                if(!E.available) continue;

                int other = (E.u==i ? E.v : E.u);

                printf("   - Connected to: %s via %s (time=%d, cost=%d)\n",
                    G.cities[other].name, E.mode, E.time, E.cost
                );
                hasConn=1;
            }

            if(!hasConn){
                printf("   - No connections yet.\n");
            }

            printf("\n");
        }
    }

    printf("========================================================\n");
}
// ===================== SMALL HELPERS =====================

void printAllCitiesShort(){
    if(G.cityCount==0){
        printf("No cities yet.\n");
        return;
    }
    printf("Cities: ");
    for(int i=0;i<G.cityCount;i++){
        printf("%s",G.cities[i].name);
        if(i<G.cityCount-1) printf(", ");
    }
    printf("\n");
}

void printFilteredMap(){
    printf("\n====== FILTERED MAP (PREFERENCES APPLIED, TIME-BASED) ======\n");
    if(G.cityCount==0){
        printf("No cities.\n");
        printf("===========================================================\n");
        return;
    }

    int prefId = modeId(S.preferredMode);

    for(int i=0;i<G.cityCount;i++){
        printf("[%s]\n",G.cities[i].name);
        int printed=0;

        for(int j=0;j<G.deg[i];j++){
            int ei=G.adj[i][j];
            if(ei<0) continue;

            Edge E=G.edges[ei];
            if(!E.available) continue;

            int mid=modeId(E.mode);
            if(mid==-1) continue;
            if(S.avoid[mid]) continue;
            if(prefId!=-1 && mid!=prefId) continue;

            int other = (E.u==i ? E.v : E.u);

            printf("  ├─ %s via %s (time=%d, cost=%d)\n",
                   G.cities[other].name,E.mode,E.time,E.cost);
            printed=1;
        }

        if(!printed) printf("  (no routes visible under current preferences)\n");
        printf("-----------------------------------------------------------\n");
    }

    printf("===========================================================\n");
}

void printRouteOnly(){
    if(lastPathLen<=0){
        printf("No route computed yet.\n");
        return;
    }

    printf("\n========= LAST ROUTE ONLY (TIME-BASED) =========\n");
    for(int i=lastPathLen-1;i>=0;i--){
        int ci=lastPathNodes[i];
        printf("%s",G.cities[ci].name);

        if(i>0){
            int e=lastPathEdges[i];
            if(e>=0){
                Edge E=G.edges[e];
                printf(" -(%s,time=%d,cost=%d)-> ",E.mode,E.time,E.cost);
            }else{
                printf(" -> ");
            }
        }
    }
    printf("\n===============================================\n");
}


// ===================== DIJKSTRA (TIME-BASED) =====================

int runDijkstraTime(int src,int dst){
    int prefId = modeId(S.preferredMode);

    for(int i=0;i<G.cityCount;i++){
        for(int m=0;m<6;m++){
            distArr[i][m]=INF;
            parentNodeArr[i][m]=-1;
            parentModeArr[i][m]=-1;
            parentEdgeArr[i][m]=-1;
        }
    }

    pqClear();
    distArr[src][NONE_MODE]=0;
    pqPush(src,NONE_MODE,0);

    while(pqSz>0){
        int u,um,ud;
        if(!pqPop(&u,&um,&ud)) break;
        if(ud!=distArr[u][um]) continue;

        for(int k=0;k<G.deg[u];k++){
            int ei=G.adj[u][k];
            if(ei<0) continue;

            Edge E=G.edges[ei];
            if(!E.available) continue;

            int mid=modeId(E.mode);
            if(mid==-1) continue;
            if(S.avoid[mid]) continue;

            int v = (E.u==u ? E.v : E.u);
            int w = E.time;

            // preference effect
            if(prefId!=-1){
                if(mid==prefId){
                    w=w*7/10;
                    if(w<=0) w=1;
                }else{
                    w=w*11/10;
                    if(w<=0) w=1;
                }
            }

            // mode switch penalty
            if(um!=NONE_MODE && um!=mid) w+=S.switchPenalty;

            int nd = ud+w;
            if(nd<distArr[v][mid]){
                distArr[v][mid]=nd;
                parentNodeArr[v][mid]=u;
                parentModeArr[v][mid]=um;
                parentEdgeArr[v][mid]=ei;
                pqPush(v,mid,nd);
            }
        }
    }

    int best=INF;
    for(int m=0;m<6;m++){
        if(distArr[dst][m]<best) best=distArr[dst][m];
    }
    return best;
}

void reconstructAndStoreTimePath(int src,int dst){
    int best=INF,bm=NONE_MODE;

    for(int m=0;m<6;m++){
        if(distArr[dst][m]<best){
            best=distArr[dst][m];
            bm=m;
        }
    }

    if(best>=INF){
        printf("No path between these cities under current preferences.\n");
        lastPathLen=0;
        return;
    }

    int pn[500],pe[500],pc=0;
    int curN=dst, curM=bm;

    while(1){
        pn[pc]=curN;

        if(parentNodeArr[curN][curM]==-1){
            pe[pc]=-1; // starting node: no incoming edge
            break;
        }

        int e=parentEdgeArr[curN][curM];
        pe[pc]=e;

        int nn=parentNodeArr[curN][curM];
        int nm=parentModeArr[curN][curM];

        curN=nn;
        curM=nm;
        pc++;
        if(pc>=499) break;
    }

    printf("\n======= SHORTEST ROUTE (TIME-BASED) =======\n");
    printf("Total time (with preferences)=%d\n",best);

    lastPathLen=pc+1;
    for(int i=pc;i>=0;i--){
        int ci=pn[i];
        lastPathNodes[i]=ci;
        printf("%s",G.cities[ci].name);

        if(i>0){
            int e=pe[i];
            lastPathEdges[i]=e;
            if(e>=0){
                Edge E=G.edges[e];
                printf(" -(%s,time=%d,cost=%d)-> ",E.mode,E.time,E.cost);
            }else{
                printf(" -> ");
            }
        }else{
            lastPathEdges[i]=-1;
        }
    }
    printf("\n===========================================\n");
}


// ===================== DIJKSTRA (COST-BASED) =====================

int runDijkstraCost(int src,int dst){
    for(int i=0;i<G.cityCount;i++){
        costDist[i]=INF;
        costParentNode[i]=-1;
        costParentEdge[i]=-1;
    }

    cpqClear();
    costDist[src]=0;
    cpqPush(src,0);

    while(cpqSz>0){
        int u,ud;
        if(!cpqPop(&u,&ud)) break;
        if(ud!=costDist[u]) continue;

        for(int k=0;k<G.deg[u];k++){
            int ei=G.adj[u][k];
            if(ei<0) continue;

            Edge E=G.edges[ei];
            if(!E.available) continue;

            int v = (E.u==u ? E.v : E.u);
            int w = E.cost;

            int nd = ud+w;
            if(nd<costDist[v]){
                costDist[v]=nd;
                costParentNode[v]=u;
                costParentEdge[v]=ei;
                cpqPush(v,nd);
            }
        }
    }

    return costDist[dst];
}

void showCheapestRouteWithBudget(){
    if(G.cityCount==0){
        printf("Graph empty.\n");
        return;
    }

    char s[32],d[32];
    int money;

    printf("Enter source city: ");
    scanf("%31s",s);
    printf("Enter destination city: ");
    scanf("%31s",d);
    printf("Enter how much money you have: ");
    scanf("%d",&money);

    int si=findCityIndex(s);
    int di=findCityIndex(d);

    if(si==-1 || di==-1){
        printf("City not found.\n");
        return;
    }

    int minCost = runDijkstraCost(si,di);
    if(minCost>=INF){
        printf("There is no possible route (graph disconnected).\n");
        return;
    }

    if(minCost>money){
        printf("Minimum required cost = %d\n",minCost);
        printf("You cannot go with this money.\n");
        return;
    }

    // reconstruct cost-based path
    int pn[500],pe[500],pc=0;
    int cur=di;

    while(1){
        pn[pc]=cur;
        if(costParentNode[cur]==-1){
            pe[pc]=-1;
            break;
        }
        int e=costParentEdge[cur];
        pe[pc]=e;
        cur=costParentNode[cur];
        pc++;
        if(pc>=499) break;
    }

    printf("\n===== CHEAPEST ROUTE (BUDGET OK) =====\n");
    printf("Total cost = %d (your money = %d)\n",minCost,money);

    for(int i=pc;i>=0;i--){
        int ci=pn[i];
        printf("%s",G.cities[ci].name);

        if(i>0){
            int e=pe[i];
            if(e>=0){
                Edge E=G.edges[e];
                printf(" -(%s,cost=%d,time=%d)-> ",E.mode,E.cost,E.time);
            }else{
                printf(" -> ");
            }
        }
    }

    printf("\n======================================\n");
}


// ===================== NEAREST HUB =====================

int isHubOfType(int city,int typeId){
    for(int j=0;j<G.deg[city];j++){
        int ei=G.adj[city][j];
        if(ei<0) continue;

        Edge E=G.edges[ei];
        if(!E.available) continue;

        int mid=modeId(E.mode);
        if(typeId==0 && mid==2) return 1; // METRO
        if(typeId==1 && mid==1) return 1; // BUS
        if(typeId==2 && mid==4) return 1; // AIR
    }
    return 0;
}

int findNearestHub(int src,int typeId){
    int dist[MAXN];
    int vis[MAXN];

    for(int i=0;i<G.cityCount;i++){
        dist[i]=INF;
        vis[i]=0;
    }

    dist[src]=0;

    for(int iter=0;iter<G.cityCount;iter++){
        int u=-1,best=INF;
        for(int i=0;i<G.cityCount;i++){
            if(!vis[i] && dist[i]<best){
                best=dist[i];
                u=i;
            }
        }

        if(u==-1) break;
        vis[u]=1;

        if(u!=src){
            if(isHubOfType(u,typeId)) return u;
        }

        for(int k=0;k<G.deg[u];k++){
            int ei=G.adj[u][k];
            if(ei<0) continue;

            Edge E=G.edges[ei];
            if(!E.available) continue;

            int mid=modeId(E.mode);
            // For nearest hub, allow ROAD and WALK as "local movement"
            if(mid!=0 && mid!=3) continue;

            int v = (E.u==u ? E.v : E.u);
            int w = E.time;

            if(dist[u]+w<dist[v]) dist[v]=dist[u]+w;
        }
    }

    return -1;
}


// ===================== COMPARE TWO PAIRS (TIME-BASED) =====================

void compareTwoPairs(){
    char s1[32],d1[32],s2[32],d2[32];

    printf("Enter first pair (src1 dest1): ");
    scanf("%31s%31s",s1,d1);

    printf("Enter second pair (src2 dest2): ");
    scanf("%31s%31s",s2,d2);

    int a=findCityIndex(s1);
    int b=findCityIndex(d1);
    int c=findCityIndex(s2);
    int d=findCityIndex(d2);

    if(a==-1 || b==-1 || c==-1 || d==-1){
        printf("One of the cities not found.\n");
        return;
    }

    int t1=runDijkstraTime(a,b);
    int t2=runDijkstraTime(c,d);

    printf("\n=== TRAVEL TIME COMPARISON (TIME-BASED) ===\n");

    if(t1>=INF) printf("%s -> %s : NO PATH\n",s1,d1);
    else printf("%s -> %s : %d\n",s1,d1,t1);

    if(t2>=INF) printf("%s -> %s : NO PATH\n",s2,d2);
    else printf("%s -> %s : %d\n",s2,d2,t2);

    if(t1<INF && t2<INF){
        if(t1<t2) printf("First route is faster by %d minutes.\n",t2-t1);
        else if(t2<t1) printf("Second route is faster by %d minutes.\n",t1-t2);
        else printf("Both routes have equal time.\n");
    }

    printf("==========================================\n");
}
// ===================== MANAGE CITIES & ROUTES =====================

void listAllEdges(){
    if(G.edgeCount==0){
        printf("No connections.\n");
        return;
    }

    printf("ID : City1 - City2 (Mode,Time,Cost,Available)\n");
    for(int i=0;i<G.edgeCount;i++){
        Edge E=G.edges[i];

        printf("%3d: %s - %s (%s,%d,%d,%s)\n",
            i,
            G.cities[E.u].name,
            G.cities[E.v].name,
            E.mode,
            E.time,
            E.cost,
            E.available?"ON":"OFF"
        );
    }
}

void actionAddCity(){
    char name[32];
    printf("Enter new city name: ");
    scanf("%31s",name);

    if(findCityIndex(name)!=-1){
        printf("City already exists.\n");
        return;
    }

    int idx=createCity(name);
    if(idx==-1){
        printf("Maximum limit reached.\n");
        return;
    }

    isDefaultCity[idx]=0;
    printf("City %s added.\n",name);
}

void actionAddConnection(){
    char a[32],b[32],m[8];
    int t;

    printf("Enter city1: "); scanf("%31s",a);
    printf("Enter city2: "); scanf("%31s",b);
    printf("Enter mode (ROAD/BUS/METRO/WALK/AIR): "); scanf("%7s",m);
    printf("Enter travel time: "); scanf("%d",&t);

    int u=getOrCreateCity(a);
    int v=getOrCreateCity(b);

    if(!isDefaultCity[u] && findCityIndex(a)==u && u>=10) isDefaultCity[u]=0;
    if(!isDefaultCity[v] && findCityIndex(b)==v && v>=10) isDefaultCity[v]=0;

    addEdgeInternal(u,v,m,t);
    printf("Connection added.\n");
}

void actionEditConnection(){
    listAllEdges();
    if(G.edgeCount==0) return;

    int id;
    printf("Enter edge ID to edit: ");
    scanf("%d",&id);

    if(id<0 || id>=G.edgeCount){
        printf("Invalid ID.\n");
        return;
    }

    Edge *E = &G.edges[id];

    printf("Editing %s - %s (%s,time=%d,cost=%d,%s)\n",
        G.cities[E->u].name,
        G.cities[E->v].name,
        E->mode,
        E->time,
        E->cost,
        E->available?"ON":"OFF"
    );

    printf("1.Change time\n");
    printf("2.Toggle availability\n");
    printf("3.Change mode\n");
    printf("4.Back\n");
    printf("Choice: ");

    int ch; scanf("%d",&ch);

    if(ch==1){
        int t;
        printf("New time: ");
        scanf("%d",&t);
        E->time=t;
        printf("Time updated.\n");

    }else if(ch==2){
        E->available = !E->available;
        printf("Availability changed to %s.\n",E->available?"ON":"OFF");

    }else if(ch==3){
        char m[8];
        printf("New mode: ");
        scanf("%7s",m);

        int mid=modeId(m);
        if(mid==-1){
            printf("Invalid mode.\n");
        }else{
            strcpy(E->mode,modeName(mid));
            E->cost=modeBaseCost(mid);
            printf("Mode updated (cost now %d).\n",E->cost);
        }
    }
}

void actionDeleteConnection(){
    listAllEdges();
    if(G.edgeCount==0) return;

    int id;
    printf("Enter edge ID to disable: ");
    scanf("%d",&id);

    if(id<0 || id>=G.edgeCount){
        printf("Invalid ID.\n");
        return;
    }

    G.edges[id].available=0;
    printf("Connection disabled.\n");
}

void submenuManageCitiesRoutes(){
    int ch;

    while(1){
        printf("\n--- Manage Cities & Routes ---\n");
        printAllCitiesShort();

        printf("1.Add City\n");
        printf("2.Add Connection\n");
        printf("3.Edit Connection\n");
        printf("4.Delete Connection\n");
        printf("5.Back\n");
        printf("Choice: ");

        scanf("%d",&ch);

        if(ch==1) actionAddCity();
        else if(ch==2) actionAddConnection();
        else if(ch==3) actionEditConnection();
        else if(ch==4) actionDeleteConnection();
        else if(ch==5) break;
        else printf("Invalid.\n");
    }
}


// ===================== TRANSPORT PREFERENCES =====================

void submenuTransportPreferences(){
    int ch;

    while(1){
        printf("\n--- Transport Preferences (Time-based) ---\n");

        printf("Preferred mode: %s\n",
            S.preferredMode[0]=='\0' ? "NONE" : S.preferredMode
        );

        printf("Avoid modes:");
        for(int i=0;i<5;i++){
            if(S.avoid[i]) printf(" %s",modeName(i));
        }
        printf("\n");

        printf("1.Set preferred mode\n");
        printf("2.Toggle avoid mode\n");
        printf("3.Clear preferences\n");
        printf("4.Back\n");
        printf("Choice: ");

        scanf("%d",&ch);

        if(ch==1){
            char m[8];
            printf("Enter preferred mode (ROAD/BUS/METRO/WALK/AIR or NONE): ");
            scanf("%7s",m);

            if(strcasecmp(m,"NONE")==0){
                S.preferredMode[0]='\0';
                printf("Preferred cleared.\n");
            }else{
                int id=modeId(m);
                if(id==-1) printf("Invalid.\n");
                else{
                    strcpy(S.preferredMode,modeName(id));
                    printf("Preferred mode set to %s.\n",S.preferredMode);
                }
            }

        }else if(ch==2){
            char m[8];
            printf("Enter mode to toggle avoid: ");
            scanf("%7s",m);

            int id=modeId(m);
            if(id==-1) printf("Invalid.\n");
            else{
                S.avoid[id] = !S.avoid[id];
                printf("%s is now %s.\n",
                    modeName(id),
                    S.avoid[id]?"AVOIDED":"ALLOWED"
                );
            }

        }else if(ch==3){
            S.preferredMode[0]='\0';
            for(int i=0;i<5;i++) S.avoid[i]=0;
            printf("Preferences cleared.\n");

        }else if(ch==4){
            break;

        }else{
            printf("Invalid.\n");
        }
    }
}


// ===================== SHORTEST ROUTE (TIME) =====================

void actionFindShortestRoute(){
    char a[32],b[32];

    printf("Enter source city: ");
    scanf("%31s",a);

    printf("Enter destination city: ");
    scanf("%31s",b);

    int s=findCityIndex(a);
    int d=findCityIndex(b);

    if(s==-1 || d==-1){
        printf("City not found.\n");
        return;
    }

    int res=runDijkstraTime(s,d);

    if(res>=INF){
        printf("No path exists.\n");
        return;
    }

    reconstructAndStoreTimePath(s,d);
}


// ===================== ADVANCED TOOLS =====================

void submenuAdvancedTools(){
    int ch;

    while(1){
        printf("\n--- Advanced Tools ---\n");
        printf("1.Show filtered map\n");
        printf("2.Find nearest hub\n");
        printf("3.Compare travel times\n");
        printf("4.Back\n");
        printf("Choice: ");

        scanf("%d",&ch);

        if(ch==1){
            printFilteredMap();

        }else if(ch==2){
            char s[32];
            int type;

            printf("Enter source city: ");
            scanf("%31s",s);

            int si=findCityIndex(s);
            if(si==-1){
                printf("City not found.\n");
                continue;
            }

            printf("Hub type (0=METRO,1=BUS,2=AIR): ");
            scanf("%d",&type);

            if(type<0 || type>2){
                printf("Invalid type.\n");
                continue;
            }

            int hub=findNearestHub(si,type);
            if(hub==-1)
                printf("No reachable %s hub.\n",
                    type==0?"METRO":(type==1?"BUS":"AIR"));
            else
                printf("Nearest %s hub from %s is %s\n",
                    type==0?"METRO":(type==1?"BUS":"AIR"),
                    s,
                    G.cities[hub].name
                );

        }else if(ch==3){
            compareTwoPairs();

        }else if(ch==4){
            break;

        }else{
            printf("Invalid.\n");
        }
    }
}




int main(){
    initGraph();
    buildDefaultCityGraph();

    // Show ASCII and added cities initially
    printAsciiStatic();
    printAddedCitiesSummary();

    printf("=============================================\n");
    printf("        SMART CITY TRANSPORT SIMULATOR       \n");
    printf("=============================================\n");

    int ch;

    while(1){
        printf("\n============= MAIN MENU =============\n");
        printAllCitiesShort();

        printf("PreferredMode = %s\n",
            S.preferredMode[0]=='\0' ? "NONE" : S.preferredMode
        );

        printf("1.Manage Cities & Routes\n");
        printf("2.Show City Map (ASCII + Added Cities)\n");
        printf("3.Transport Preferences\n");
        printf("4.Find Shortest Route (time)\n");
        printf("5.Show Last Route\n");
        printf("6.Advanced Tools\n");
        printf("7.Find Cheapest Route (budget)\n");
        printf("8.Exit\n");
        printf("Choice: ");

        scanf("%d",&ch);

        if(ch==1) submenuManageCitiesRoutes();
        else if(ch==2){
            printAsciiStatic();
            printAddedCitiesSummary();
        }
        else if(ch==3) submenuTransportPreferences();
        else if(ch==4) actionFindShortestRoute();
        else if(ch==5) printRouteOnly();
        else if(ch==6) submenuAdvancedTools();
        else if(ch==7) showCheapestRouteWithBudget();
        else if(ch==8) break;
        else printf("Invalid.\n");
    }

    printf("Exiting simulator.\n");
    return 0;
}
