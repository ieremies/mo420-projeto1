//----------------------------------------------------------------------
// Projeto 1 da disciplina Programacao Linear Inteira, utilizando as bibliotecas
// Lemon, Graphviz e Gurobi
//
// Send comments/corrections to Flavio K. Miyazawa.
//----------------------------------------------------------------------
#include "mygraphlib.h"
#include "myutils.h"
#include "solver.h"
#include <assert.h>
#include <float.h>
#include <gurobi_c++.h>
#include <lemon/adaptors.h>
#include <lemon/connectivity.h>
#include <lemon/dijkstra.h>
#include <lemon/gomory_hu.h>
#include <lemon/list_graph.h>
#include <lemon/unionfind.h>
#include <math.h>
#include <set>

// This is the type used to obtain the pointer to the problem data. This pointer
// is stored in the branch and cut tree. And when we define separation routines,
// we can recover the pointer and access the problem data again.
class Drone_Data {
public:
  Drone_Data(Digraph &dg, DNodeStringMap &nodename, DNodePosMap &posicaox,
             DNodePosMap &posicaoy, ArcValueMap &car_dist,
             ArcValueMap &drone_dist, DNode &source_node, DNode &target_node,
             double &drone_limite);

  Digraph &dg;
  int nnodes, narcs;
  DNodeStringMap &vname;
  ArcStringMap aname;
  DNodeColorMap vcolor;
  ArcColorMap acolor;
  ArcValueMap &car_cost;
  ArcValueMap &drone_cost;
  DNodePosMap &posx;
  DNodePosMap &posy;
  DNode &source, &target;
  double &drone_limit;
};

Drone_Data::Drone_Data(Digraph &dig, DNodeStringMap &nodename,
                       DNodePosMap &posicaox, DNodePosMap &posicaoy,
                       ArcValueMap &car_dist, ArcValueMap &drone_dist,
                       DNode &source_node, DNode &target_node,
                       double &drone_limite)
    : dg(dig), vname(nodename), aname(dig), vcolor(dig), acolor(dig),
      car_cost(car_dist), drone_cost(drone_dist), posx(posicaox),
      posy(posicaoy), source(source_node), target(target_node),
      drone_limit(drone_limite) {
  nnodes = countNodes(this->dg);
  narcs = countArcs(this->dg);
}

bool Read_Proj1_Instance(string filename, Digraph &dg, DNodeStringMap &vname,
                         DNodePosMap &px, DNodePosMap &py,
                         ArcValueMap &car_cost, ArcValueMap &drone_cost,
                         DNode &source, DNode &target, double &drone_limit) {
  DigraphTable DT(filename, dg);
  string sourcename, targetname;
  int nnodes, narcs;
  bool r = (DT.Get("nnodes", nnodes) && DT.Get("narcs", narcs) &&
            DT.Get("source", sourcename) && DT.Get("target", targetname) &&
            DT.Get("drone_limit", drone_limit) && DT.Get("nodename", vname) &&
            DT.Get("posx", px) && DT.Get("posy", py) &&
            DT.Get("car_cost", car_cost) && DT.Get("drone_cost", drone_cost));
  if (!r)
    return (r);

  // cout << "nnodes = " << nnodes << endl;
  // cout << "narcs = " << narcs << endl;
  // cout << "sourcename = " << sourcename << endl;
  // cout << "targetname = " << targetname << endl;
  // cout << "drone_limit = " << drone_limit << endl;

  // for (DNodeIt v(dg); v!=INVALID; ++v) {
  //   cout << vname[v] << " " << px[v] << " " << py[v] << endl;}

  // for (ArcIt a(dg); a!=INVALID; ++a) {
  //   cout << vname[dg.source(a)] << " " << vname[dg.target(a)] << " " <<
  //   car_cost[a] << " " << drone_cost[a] << endl;}

  source = INVALID;
  for (DNodeIt v(dg); v != INVALID; ++v) {
    if (vname[v] == sourcename) {
      source = v;
      break;
    }
  }
  if (source == INVALID) {
    cout << "ERROR: Could not find source node \"" << sourcename << "\"."
         << endl;
    return (false);
  }
  target = INVALID;
  for (DNodeIt v(dg); v != INVALID; ++v) {
    if (vname[v] == targetname) {
      target = v;
      break;
    }
  }
  if (target == INVALID) {
    cout << "ERROR: Could not find target node \"" << targetname << "\"."
         << endl;
    return (false);
  }
  return (r);
}

// TODO Faca um algoritmo exato, trocando a construcao fake por uma que usa
// formulacao inteira.
bool Exact_Algorithm(Drone_Data &D, DNodeArcMap &car_route_predArc,
                     DNodeArcMap &drone_in, DNodeArcMap &drone_out) {
  // Construindo uma solucao fake. Gera um caminho minimo de
  // source ate' target e depois pega um arco qualquer entrando
  // e outro arco qualquer saindo, para cada um dos outros vertices.
  Dijkstra<ListDigraph, ArcValueMap> dijkstra(D.dg, D.car_cost);
  DNodeValueMap dist(D.dg);
  dijkstra.distMap(dist);
  dijkstra.init();
  dijkstra.addSource(D.source);
  dijkstra.start();

  DNodeBoolMap car_visit(D.dg);
  for (DNodeIt v(D.dg); v != INVALID; ++v)
    car_visit[v] = false;
  DNode v;
  Arc a;
  v = D.target;
  car_visit[v] = true;
  while (v != D.source) {
    car_route_predArc[v] = dijkstra.predArc(v);
    v = D.dg.source(car_route_predArc[v]);
    car_visit[v] = true;
  }

  for (DNodeIt v(D.dg); v != INVALID; ++v) {
    if (!car_visit[v]) {
      // Pega o primeiro arco que entra e o primeiro que sai
      // Obviamente esta' errado, mas so' para ver alguma atribuicao de arestas
      // p/ solucao
      for (InArcIt a(D.dg, v); a != INVALID; ++a) {
        drone_in[v] = a;
        break;
      }
      for (OutArcIt a(D.dg, v); a != INVALID; ++a) {
        drone_out[v] = a;
        break;
      }
    }
  }
}

// Faca uma heuristica, trocando a construcao fake por uma heuristica
bool Heuristic_Algorithm(Drone_Data &D, DNodeArcMap &car_route_predArc,
                         DNodeArcMap &drone_in, DNodeArcMap &drone_out) {
  // Construindo uma solucao fake. Gera um caminho minimo de
  // source ate' target e depois pega um arco qualquer entrando
  // e outro arco qualquer saindo, para cada um dos outros vertices.
  Dijkstra<ListDigraph, ArcValueMap> dijkstra(D.dg, D.car_cost);
  DNodeValueMap dist(D.dg);
  dijkstra.distMap(dist);
  dijkstra.init();
  dijkstra.addSource(D.source);
  dijkstra.start();

  DNodeBoolMap car_visit(D.dg);
  for (DNodeIt v(D.dg); v != INVALID; ++v)
    car_visit[v] = false;
  DNode v;
  Arc a;
  v = D.target;
  car_visit[v] = true;
  while (v != D.source) {
    car_route_predArc[v] = dijkstra.predArc(v);
    v = D.dg.source(car_route_predArc[v]);
    car_visit[v] = true;
  }

  for (DNodeIt v(D.dg); v != INVALID; ++v) {
    if (!car_visit[v]) {
      // Pega o primeiro arco que entra e o primeiro que sai
      // Obviamente esta' errado, mas so' para ver alguma atribuicao de arestas
      // p/ solucao
      for (InArcIt a(D.dg, v); a != INVALID; ++a) {
        drone_in[v] = a;
        break;
      }
      for (OutArcIt a(D.dg, v); a != INVALID; ++a) {
        drone_out[v] = a;
        break;
      }
    }
  }
}

bool View_Car_Drone_Routing(Drone_Data &D, DNodeArcMap &car_route_predArc,
                            DNodeArcMap &drone_in, DNodeArcMap &drone_out) {
  // Apenas para visualizar a solucao gerada
  DigraphAttributes DA(D.dg, D.vname, D.posx, D.posy);
  DA.SetDigraphAttrib("splines=false");
  DA.SetDefaultDNodeAttrib("color=Gray style=filled width=0.2 height=0.2");
  DA.SetDefaultArcAttrib("color=Black penwidth=3");
  DA.SetColor(D.source, "Red");
  DA.SetColor(D.target, "Blue");

  double custo_total = 0.0;

  DNodeBoolMap car_visit(D.dg);
  for (DNodeIt v(D.dg); v != INVALID; ++v)
    car_visit[v] = false;
  DNode v;
  Arc a;
  v = D.target;
  car_visit[v] = true;
  while (v != D.source) {
    Arc a = car_route_predArc[v];
    custo_total += D.car_cost[a];
    v = D.dg.source(a);
    car_visit[v] = true;
  }

  v = D.target;
  while (v != D.source) {
    Arc a = car_route_predArc[v];
    DA.SetLabel(a, D.car_cost[a]);
    DA.SetFontSize(a, 20);
    cout << "( " << D.vname[D.dg.source(a)] << " --> "
         << D.vname[D.dg.target(a)] << ")" << endl;
    DA.SetColor(a, "Blue");
    v = D.dg.source(car_route_predArc[v]);
  }

  // Pinta de verde os arcos usados pelo drone
  for (DNodeIt v(D.dg); v != INVALID; ++v) {
    if (!car_visit[v]) { // deve ter sido atendido pelo drone
      custo_total += D.drone_cost[drone_in[v]];
      custo_total += D.drone_cost[drone_out[v]];
      DA.SetColor(drone_in[v], "Green");
      DA.SetLabel(drone_in[v], D.drone_cost[drone_in[v]]);
      DA.SetColor(drone_out[v], "Green");
      DA.SetLabel(drone_out[v], D.drone_cost[drone_out[v]]);
      DA.SetColor(v, "Cyan");
    }
  }
  DA.SetLabel("Solucao fake de valor " + DoubleToString(custo_total));
  DA.View();
}

int main(int argc, char *argv[]) {
  int time_limit;
  char name[1000];
  Digraph dg;
  DNode source, target;
  ArcValueMap car_cost(dg), drone_cost(dg);
  DNodeStringMap vname(dg);
  DNodePosMap posx(dg), posy(dg);
  string filename;
  double drone_limit;

  int seed = 1;

  // uncomment one of these lines to change default pdf reader, or insert new
  // one
  set_pdfreader("open"); // pdf reader for Mac OS X
  // set_pdfreader("xpdf");    // pdf reader for Linux
  // set_pdfreader("evince");  // pdf reader for Linux

  srand48(seed);
  time_limit = 3600; // solution must be obtained within time_limit seconds
  if (argc != 2) {
    cout << endl
         << "Usage: " << argv[0] << " <pli1_filename> " << endl
         << endl
         << "Example:" << endl
         << "\t" << argv[0] << " "
         << getpath(argv[0]) + "../instances/pli1_10_1000_1000_1_0.2_0.25"
         << endl
         << "\t" << argv[0] << " "
         << getpath(argv[0]) + "../instances/pli1_30_1000_1000_1_0.2_0.5"
         << endl
         << endl;
    exit(0);
  }

  else if (!FileExists(argv[1])) {
    cout << "File " << argv[1] << " does not exist." << endl;
    exit(0);
  }
  filename = argv[1];

  Read_Proj1_Instance(filename, dg, vname, posx, posy, car_cost, drone_cost,
                      source, target, drone_limit);
  Drone_Data D(dg, vname, posx, posy, car_cost, drone_cost, source, target,
               drone_limit);

  DNodeArcMap car_route_predArc(D.dg);
  DNodeArcMap drone_out(D.dg), drone_in(D.dg);

  Exact_Algorithm(D, car_route_predArc, drone_in, drone_out);

  View_Car_Drone_Routing(D, car_route_predArc, drone_in, drone_out);
}
