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

class subtourelim : public GRBCallback {
  Drone_Data &drone;
  Digraph::ArcMap<GRBVar> &x;
  double (GRBCallback::*solution_value)(GRBVar);

public:
  subtourelim(Drone_Data &drone, Digraph::ArcMap<GRBVar> &x)
      : drone(drone), x(x) {}

protected:
  void callback() {
    // --------------------------------------------------------------------------------
    // get the correct function to obtain the values of the lp variables
    // if this condition is true, all variables are integer
    if (where == GRB_CB_MIPSOL) {
      solution_value = &subtourelim::getSolution;
    } else if ((where == GRB_CB_MIPNODE) &&
               (getIntInfo(GRB_CB_MIPNODE_STATUS) ==
                GRB_OPTIMAL)) // node with optimal fractional solution
    {
      solution_value = &subtourelim::getNodeRel;
    } else
      return; // return, as this code do not take advantage of the other
              // options

    // --------------------------------------------------------------------------------
    // Stores the edges with fractional values and integer values
    vector<Arc> FracArcs, OneArcs;
    // produces a subgraph h of g, with arcs a with x[a]==1
    // contracted, so we can apply Gomory-Hu tree in a small graph
    for (ArcIt e(drone.dg); e != INVALID; ++e) {
      if ((this->*solution_value)(x[e]) > 1 - MY_EPS)
        OneArcs.push_back(e); // stores the edges with x[e]==1
      else if ((this->*solution_value)(x[e]) > MY_EPS)
        FracArcs.push_back(e); // includes edges with 0 < x[e] < 1
    } // define the subgraph with edges that have x[e]==1

    try {
      // --------------------------------------------------------------------------------
      // Use union-find to contract nodes (to obtain graph where each component
      // of g is contracted)
      Digraph::NodeMap<int> aux_map(drone.dg);
      UnionFind<Digraph::NodeMap<int>> UFNodes(aux_map);
      for (Digraph::NodeIt v(drone.dg); v != INVALID; ++v)
        UFNodes.insert(v);
      for (auto a : OneArcs)
        UFNodes.join(drone.dg.source(a), drone.dg.target(a));
      // --------------------------------------------------------------------------------
      // Put in a separate set all edges that are not inside a component
      vector<Arc> CrossingArcs;
      for (ArcIt e(drone.dg); e != INVALID; ++e)
        if (UFNodes.find(drone.dg.source(e)) !=
            UFNodes.find(drone.dg.target(e)))
          CrossingArcs.push_back(e);
      // --------------------------------------------------------------------------------
      // Generate an inverted list UFIndexToNode to find the node that
      // represents a component
      vector<bool> ComponentIndex(drone.nnodes);
      vector<Graph::Node> Index2h(drone.nnodes);
      for (int i = 0; i < drone.nnodes; i++)
        ComponentIndex[i] = false;
      for (Digraph::NodeIt v(drone.dg); v != INVALID; ++v)
        ComponentIndex[UFNodes.find(v)] = true;
      // --------------------------------------------------------------------------------
      // Generate graph of components, add one node for each component and edges
      Graph h;
      EdgeValueMap h_capacity(h);
      for (int i = 0; i < drone.nnodes; i++) // add nodes to the graph h
        if (ComponentIndex[i])
          Index2h[i] = h.addNode();
      for (auto a : FracArcs) {
        Digraph::Node u = drone.dg.source(a), v = drone.dg.target(a);
        Node hu = Index2h[UFNodes.find(u)], hv = Index2h[UFNodes.find(v)];
        Edge e = h.addEdge(hu, hv); // add edges to the graph h
        h_capacity[e] = (this->*solution_value)(x[a]);
      }
      // --------------------------------------------------------------------------------
      GomoryHu<Graph, EdgeValueMap> ght(h, h_capacity);
      ght.run();

      // --------------------------------------------------------------------------------
      // The Gomory-Hu tree is given as a rooted directed tree. Each node has
      // an arc that points to its father. The root node has father -1.
      // Remember that each arc in this tree represents a cut and the value of
      // the arc is the weight of the corresponding cut. So, if an arc has
      // weight less than 2, then we found a violated cut and in this case, we
      // insert the corresponding constraint.

      NodeBoolMap cutmap(h);
      for (Graph::NodeIt u(h); u != INVALID; ++u) {
        GRBLinExpr expr = 0;
        if (ght.predNode(u) == INVALID)
          continue; // skip the root node
        if (ght.predValue(u) > 2.0 - MY_EPS)
          continue; // value of the cut is good
        ght.minCutMap(u, ght.predNode(u),
                      cutmap); // now, we have a violated cut

        // Percorre as arestas que cruzam alguma componente e insere as que
        // pertencem ao corte
        for (auto e_it : CrossingArcs) {
          Digraph::Node u = drone.dg.source(e_it), v = drone.dg.target(e_it);
          Node hu = Index2h[UFNodes.find(u)], hv = Index2h[UFNodes.find(v)];
          if (cutmap[hu] != cutmap[hv])
            expr += x[e_it];
        }
        addLazy(expr >= 2);
      }
    } catch (...) {
      cout << "Error during callback..." << endl;
    }
  }
};

// Faca um algoritmo exato, trocando a construcao fake por uma que usa
// formulacao inteira.
bool Exact_Algorithm(Drone_Data &D, DNodeArcMap &car_route_predArc,
                     DNodeArcMap &drone_in, DNodeArcMap &drone_out) {

  string aux;

  Digraph::ArcMap<GRBVar> x(D.dg);  // arestas usadas pelo caminhão
  Digraph::ArcMap<GRBVar> y(D.dg);  // arestas usadas pelo drone
  Digraph::NodeMap<GRBVar> u(D.dg); //

  GRBEnv env = GRBEnv();
  env.set(GRB_DoubleParam_TimeLimit, 300);
  GRBModel model = GRBModel(env);
  GRBLinExpr obj;
  model.set(GRB_StringAttr_ModelName, "DRONE-D");
  model.set(GRB_IntAttr_ModelSense, GRB_MINIMIZE);
  model.getEnv().set(GRB_IntParam_LazyConstraints, 1);

  // Soma dos custos das arestas utilizadas pelo caminhão
  for (ArcIt e(D.dg); e != INVALID; ++e) {
    x[e] = model.addVar(0.0, 1.0, 0.0, GRB_BINARY, "x");
    y[e] = model.addVar(0.0, 1.0, 0.0, GRB_BINARY, "y");
    obj += x[e] * D.car_cost[e] + y[e] * D.drone_cost[e];
  }

  for (DNodeIt n(D.dg); n != INVALID; ++n)
    u[n] = model.addVar(0.0, D.nnodes, 0.0, GRB_CONTINUOUS, "u");

  model.setObjective(obj, GRB_MINIMIZE);
  model.update();

  // Source
  GRBLinExpr c_source_in;
  for (InArcIt e(D.dg, D.source); e != INVALID; ++e)
    c_source_in += x[e];
  model.addConstr(c_source_in == 0);
  GRBLinExpr c_source_out;
  for (OutArcIt e(D.dg, D.source); e != INVALID; ++e)
    c_source_out += x[e];
  model.addConstr(c_source_out == 1);

  // Target
  GRBLinExpr c_target_in;
  for (InArcIt e(D.dg, D.target); e != INVALID; ++e)
    c_target_in += x[e];
  model.addConstr(c_target_in == 1);
  GRBLinExpr c_target_out;
  for (OutArcIt e(D.dg, D.target); e != INVALID; ++e)
    c_target_out += x[e];
  model.addConstr(c_target_out == 0);

  // manutenção de fluxo
  for (DNodeIt n(D.dg); n != INVALID; ++n) {
    if (n == D.source or n == D.target)
      continue;
    GRBLinExpr c_fluxo;
    for (InArcIt e(D.dg, n); e != INVALID; ++e)
      c_fluxo += x[e];
    for (OutArcIt e(D.dg, n); e != INVALID; ++e)
      c_fluxo -= x[e];
    model.addConstr(c_fluxo == 0);
  }

  // subciclo
  for (ArcIt a(D.dg); a != INVALID; ++a)
    if (D.dg.target(a) != D.source)
      model.addConstr(u[D.dg.source(a)] - u[D.dg.target(a)] + D.nnodes * x[a] <=
                      D.nnodes - 1);

  // ida e volta do drone dentro do limite dele
  for (ArcIt a(D.dg); a != INVALID; ++a)
    for (ArcIt b(D.dg); b != INVALID; ++b)
      if (D.dg.source(a) == D.dg.target(b) and
          D.dg.target(a) == D.dg.source(b)) {
        model.addConstr(y[a] == y[b]);
        model.addConstr(y[a] * D.drone_cost[a] + y[b] * D.drone_cost[b] <=
                        D.drone_limit);
      }

  // Para todo arco do drone, deve haver um arco do caminhão que chega nele
  for (ArcIt a(D.dg); a != INVALID; ++a) {
    GRBLinExpr c;
    for (InArcIt e(D.dg, D.dg.source(a)); e != INVALID; ++e)
      c += x[e];
    for (InArcIt e(D.dg, D.dg.target(a)); e != INVALID; ++e)
      c += x[e];
    model.addConstr(c >= y[a]);
  }

  // Todos os vértices devem ser visitados ao menos uma vez
  for (DNodeIt n(D.dg); n != INVALID; ++n) {
    if (n == D.source or n == D.target)
      continue;
    GRBLinExpr c;
    for (InArcIt e(D.dg, n); e != INVALID; ++e)
      c += x[e] + y[e]; // TODO eu acho que não precisa de x[e]
    model.addConstr(c >= 1);
  }

  subtourelim cb = subtourelim(D, x);
  model.setCallback(&cb);

  model.optimize();

  for (ArcIt e(D.dg); e != INVALID; ++e) {
    if (x[e].get(GRB_DoubleAttr_X) > 0) {
      car_route_predArc[D.dg.target(e)] = e;
      cout << "C\t" << D.vname[D.dg.source(e)] << "\t-->\t"
           << D.vname[D.dg.target(e)] << "\t" << D.car_cost[e] << endl;
    }
    if (y[e].get(GRB_DoubleAttr_X) > 0) {
      drone_in[D.dg.target(e)] = e;
      drone_out[D.dg.source(e)] = e;
      cout << "D\t" << D.vname[D.dg.source(e)] << "\t-->\t"
           << D.vname[D.dg.target(e)] << "\t" << D.drone_cost[e] << endl;
    }
  }

  return true;
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
  DA.SetLabel("Solucao de valor " + DoubleToString(custo_total));
  DA.View();

  return true;
}

int main(int argc, char *argv[]) {
  // int time_limit;
  // char name[1000];
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
  set_pdfreader("okular"); // pdf reader for Mac OS X
  // set_pdfreader("xpdf");    // pdf reader for Linux
  // set_pdfreader("evince");  // pdf reader for Linux

  srand48(seed);
  // time_limit = 3600; // solution must be obtained within time_limit seconds
  if (argc != 3) {
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
  // View_Car_Drone_Routing(D, car_route_predArc, drone_in, drone_out);
}
