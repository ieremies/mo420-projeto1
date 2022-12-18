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
#include <lemon/list_graph.h>
#include <lemon/preflow.h>
#include <lemon/unionfind.h>
#include <math.h>
#include <set>

int PROJ2 = 1;

// This is the type used to obtain the pointer to the problem data. This pointer
// is stored in the branch and cut tree. And when we define separation routines,
// we can recover the pointer and access the problem data again.
class Drone_Data {
public:
  Drone_Data(Digraph &dg, DNodeStringMap &nodename, DNodePosMap &posicaox,
             DNodePosMap &posicaoy, ArcValueMap &car_dist,
             ArcValueMap &drone_dist, DNode &source_node, DNode &target_node);

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
};

Drone_Data::Drone_Data(Digraph &dig, DNodeStringMap &nodename,
                       DNodePosMap &posicaox, DNodePosMap &posicaoy,
                       ArcValueMap &car_dist, ArcValueMap &drone_dist,
                       DNode &source_node, DNode &target_node)
    : dg(dig), vname(nodename), aname(dig), vcolor(dig), acolor(dig),
      car_cost(car_dist), drone_cost(drone_dist), posx(posicaox),
      posy(posicaoy), source(source_node), target(target_node) {
  nnodes = countNodes(this->dg);
  narcs = countArcs(this->dg);
}

bool Read_Proj_Instance(string filename, Digraph &dg, DNodeStringMap &vname,
                        DNodePosMap &px, DNodePosMap &py, ArcValueMap &car_cost,
                        ArcValueMap &drone_cost, DNode &source, DNode &target) {
  DigraphTable DT(filename, dg);
  string sourcename, targetname;
  int nnodes, narcs;
  bool r =
      (DT.Get("nnodes", nnodes) && DT.Get("narcs", narcs) &&
       DT.Get("source", sourcename) && DT.Get("target", targetname) &&
       DT.Get("nodename", vname) && DT.Get("posx", px) && DT.Get("posy", py) &&
       DT.Get("car_cost", car_cost) && DT.Get("drone_cost", drone_cost));
  if (!r)
    return (r);

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
    // -----------------------------------------------------------------

    bool found = false;   // se um subciclo foi achado
    vector<Arc> subcycle; // o caminho atual

    for (DNodeIt o(drone.dg); o != INVALID and !found; ++o) {
      // Inicializa as estruturas necessárias
      DNodeBoolMap visited(drone.dg, false);
      subcycle.clear();

      // Nó inicial
      DNode n = o;
      visited[n] = true;

      // Caminha pelo grafo
      for (Arc a = next(n); a != INVALID and !found; a = next(n)) {
        subcycle.push_back(a);
        n = drone.dg.target(a);

        // Se voltarmos para alguém já visitado, achamos um subciclo
        if (visited[n])
          found = true;
        visited[n] = true;
      }
    }

    if (found) {
      vector<DNode> comp;
      bool flag = false;
      for (Arc a : subcycle)
        comp.push_back(drone.dg.source(a));

      GRBLinExpr expr;

      // Estratégia 1
      if (PROJ2 == 1) {
        for (Arc a : subcycle)
          expr += x[a];
      }
      // Estratégia 2
      if (PROJ2 == 2) {
        for (int i = 0; i < comp.size(); i++)
          for (int j = 0; j < comp.size(); j++) {
            if (i == j)
              continue;
            Arc a = findArc(drone.dg, comp[i], comp[j]);
            if (a != INVALID and drone.car_cost[a] > 0)
              expr += x[a];
          }
      }

      addLazy(expr <= (comp.size() - 1));
    }

    return;
  }

  Arc next(DNode &n) {
    for (OutArcIt a(drone.dg, n); a != INVALID; ++a)
      if ((this->*solution_value)(x[a]) > 1 - MY_EPS)
        return a;
    return INVALID;
  }
};

bool Exact_Algorithm(Drone_Data &D, DNodeArcMap &car_route_predArc,
                     DNodeArcMap &drone_arc) {

  Digraph::ArcMap<GRBVar> x(D.dg);  // arestas usadas pelo caminhão
  Digraph::ArcMap<GRBVar> y(D.dg);  // arestas usadas pelo drone
  Digraph::NodeMap<GRBVar> u(D.dg); //

  GRBEnv env = GRBEnv();
  env.set(GRB_DoubleParam_TimeLimit, 300);
  if (PROJ2)
    env.set(GRB_IntParam_LazyConstraints, 1);

  GRBModel model = GRBModel(env);
  model.set(GRB_StringAttr_ModelName, "DRONE-D");
  model.set(GRB_IntAttr_ModelSense, GRB_MINIMIZE);

  GRBLinExpr obj;
  GRBVar dummy_variable = model.addVar(0.0, 0.0, 0.0, GRB_BINARY, "");
  // Soma dos custos das arestas utilizadas pelo caminhão
  for (ArcIt e(D.dg); e != INVALID; ++e) {
    if (D.car_cost[e] >= 0) {
      x[e] = model.addVar(0.0, 1.0, 0.0, GRB_BINARY, "x");
      obj += x[e] * D.car_cost[e];
    } else
      x[e] = dummy_variable;

    if (D.drone_cost[e] >= 0) {
      y[e] = model.addVar(0.0, 1.0, 0.0, GRB_BINARY, "y");
      obj += y[e] * D.drone_cost[e];
    } else
      y[e] = dummy_variable;
  }

  if (!PROJ2)
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
  if (!PROJ2)
    for (ArcIt a(D.dg); a != INVALID; ++a)
      if (D.dg.target(a) != D.source)
        model.addConstr(u[D.dg.source(a)] - u[D.dg.target(a)] +
                            D.nnodes * x[a] <=
                        D.nnodes - 1);

  // Para todo arco do drone, deve haver um arco do caminhão que chega nele
  for (ArcIt a(D.dg); a != INVALID; ++a) {
    if (D.dg.source(a) == D.source)
      continue;
    GRBLinExpr c;
    for (InArcIt e(D.dg, D.dg.source(a)); e != INVALID; ++e)
      c += x[e];
    model.addConstr(c >= y[a]);
  }

  // Todos os vértices devem ser visitados ao menos uma vez
  for (DNodeIt n(D.dg); n != INVALID; ++n) {
    if (n == D.source or n == D.target)
      continue;
    GRBLinExpr c;
    for (InArcIt e(D.dg, n); e != INVALID; ++e)
      c += x[e] + y[e];
    model.addConstr(c >= 1);
  }

  if (PROJ2) {
    subtourelim cb = subtourelim(D, x);
    model.setCallback(&cb);
  }

  model.optimize();

  for (ArcIt e(D.dg); e != INVALID; ++e) {
    if (x[e].get(GRB_DoubleAttr_X) > 0) {
      car_route_predArc[D.dg.target(e)] = e;
      // cout << "C\t" << D.vname[D.dg.source(e)] << "\t-->\t"
      //      << D.vname[D.dg.target(e)] << "\t" << D.car_cost[e] << endl;
    }
    if (y[e].get(GRB_DoubleAttr_X) > 0) {
      drone_arc[D.dg.target(e)] = e;
      // cout << "D\t" << D.vname[D.dg.source(e)] << "\t-->\t"
      //      << D.vname[D.dg.target(e)] << "\t" << D.drone_cost[e] << endl;
    }
  }

  return true;
}

int compute_sol_cost(Drone_Data &drone, vector<DNode> &path) {
  DNodeBoolMap visited(drone.dg, false);
  int cost = 0;
  for (int i = 0; i < path.size() - 1; i++) {
    Arc a = findArc(drone.dg, path[i], path[i + 1]);
    if (a == INVALID || drone.car_cost[a] < 0)
      return 999999;
    cost += drone.car_cost[a];
    visited[drone.dg.source(a)] = true;
  }
  visited[path[path.size() - 1]] = true;

  for (DNodeIt n(drone.dg); n != INVALID; ++n) {
    if (visited[n])
      continue;

    int min_cost = 999999;
    for (auto u : path) {
      Arc a = findArc(drone.dg, n, u);
      if (a == INVALID || drone.car_cost[a] < 0)
        continue;
      if (drone.drone_cost[a] < min_cost)
        min_cost = drone.car_cost[a];
    }

    if (min_cost == 999999) {
      cout << "CU!\n" << endl;
      return 999999;
    }

    cost += min_cost;
  }
  return cost;
}

int heu2(Drone_Data &drone) {

  Digraph::ArcMap<int> map(drone.dg);
  for (ArcIt a(drone.dg); a != INVALID; ++a)
    if (drone.car_cost[a] >= 0)
      map[a] = drone.car_cost[a];
    else {
      map[a] = 999999;
      drone.car_cost[a] = 999999;
    }

  Dijkstra<ListDigraph, Digraph::ArcMap<int>> dk(drone.dg, map);

  dk.init();
  dk.addSource(drone.source);
  dk.run(drone.source, drone.target);

  cout << dk.dist(drone.target) << endl;

  vector<DNode> path;
  DNode n = drone.target;
  path.insert(path.begin(), n);

  for (Arc a = dk.predArc(n); a != INVALID; a = dk.predArc(n)) {
    n = drone.dg.source(a);
    path.insert(path.begin(), n);
  }

  int current_cost = compute_sol_cost(drone, path);

  // Para cada nó e cada posição,
  // eu olho se ele entrar no caminho melhora ou não o custo
  while (current_cost >= 999999)
    for (DNodeIt n(drone.dg); n != INVALID; ++n) {
      if (find(path.begin(), path.end(), n) != path.end())
        continue;

      int melhor_posicao = 1;
      int melhor_custo = INFINITY;
      vector<DNode> teste;

      for (int i = 1; i < path.size(); i++) {
        teste = path;
        teste.insert(teste.begin() + i, n);
        int teste_cost = compute_sol_cost(drone, teste);
        if (teste_cost < melhor_custo) {
          melhor_custo = teste_cost;
          melhor_posicao = i;
        }
        teste.clear();
      }

      path.insert(path.begin() + melhor_posicao, n);
      current_cost = melhor_custo;
    }
  return compute_sol_cost(drone, path);
}
bool View_Car_Drone_Routing(Drone_Data &D, DNodeArcMap &car_route_predArc,
                            DNodeArcMap &drone_arc) {
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

  // Pinta primeiro os arcos de Invisivel. E mais para frente os arcos da
  // solucao sao pintados de outra cor.
  for (ArcIt a(D.dg); a != INVALID; ++a)
    DA.SetColor(a, "Invis");

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
      custo_total += D.drone_cost[drone_arc[v]];
      DA.SetColor(drone_arc[v], "Green");
      DA.SetLabel(drone_arc[v], D.drone_cost[drone_arc[v]]);
      DA.SetColor(v, "Cyan");
    }
  }
  DA.SetLabel("Solucao fake de valor " + DoubleToString(custo_total));
  DA.View();

  return true;
}

int main(int argc, char *argv[]) {
  Digraph dg;
  DNode source, target;
  ArcValueMap car_cost(dg), drone_cost(dg);
  DNodeStringMap vname(dg);
  DNodePosMap posx(dg), posy(dg);
  string filename;

  int seed = 1;
  srand48(seed);
  set_pdfreader("zathura"); // pdf reader for Linux

  PROJ2 = atoi(argv[2]);

  filename = argv[1];

  Read_Proj_Instance(filename, dg, vname, posx, posy, car_cost, drone_cost,
                     source, target);
  Drone_Data D(dg, vname, posx, posy, car_cost, drone_cost, source, target);

  DNodeArcMap car_route_predArc(D.dg);
  DNodeArcMap drone_arc(D.dg);
  cout << heu2(D);
  // Exact_Algorithm(D, car_route_predArc, drone_arc);
  // View_Car_Drone_Routing(D, car_route_predArc, drone_arc);
}
