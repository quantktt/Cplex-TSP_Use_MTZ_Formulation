#include <iostream>
#include <ilcplex/ilocplex.h>
#include <vector>
#include <cstdlib>
#include <ctime>
#include <math.h>
using namespace std;
ILOSTLBEGIN;

int main()
{
    freopen("/home/quan/Desktop/Ai_Do?/ORProject/TestDataFile/TSP/ulysses22.tsp", "rt", stdin);
    freopen("solution_for_ulysses22MTZ.txt", "wt", stdout);
    ios::sync_with_stdio(false);
    cin.tie(0), cout.tie(0);

    int n;
    cin>>n;

    vector<double> xPos(n), yPos(n);
    for(int i=0; i<n; i++) {
        int index;
        cin>>index;
        cin>>xPos[i]>>yPos[i];
    }

    vector<vector<double>> cost(n, vector<double>(n));
    for(int i=0; i<n; i++) {
        for(int j=0; j<n; j++) {
            cost[i][j] = sqrt(pow(xPos[j]-xPos[i], 2) + pow(yPos[j]-yPos[i], 2));
        }
    }


    IloEnv env;
    try{
        IloModel model(env);

        /*----DECISION VARIABLE------------------*/
        // x[i][j] = 1 if place j is visited immediately after i, 0 otherwise
        IloArray<IloNumVarArray> x(env, n);
        for(int i=0; i<n; i++) {
            x[i] = IloNumVarArray(env, n);
            for(int j=0; j<n; j++) {
                x[i][j] = IloNumVar(env, 0, 1, ILOBOOL);
            }
        }


        /*----OBJECTIVE FUNCTION-------------------------*/
        IloExpr obj(env);
        for(int i=0; i<n; i++) {
            for(int j=0; j<n; j++) {
                if(i!=j) {
                    obj += cost[i][j]*x[i][j];
                }
            }
        }
        model.add(IloMinimize(env , obj));
        obj.end();


        /*-----CONSTRAINTS----------------------------------*/
        // for each i, the sum of all x[i][j] (i!=j) must equal 1
        IloExpr expr(env);
        for(int i=0; i<n; i++) {
            for(int j=0; j<n; j++) {
                if(i!=j) {
                    expr += x[i][j];
                }
            }
            model.add(expr == 1);
            expr.clear();
        }
        // for each j, the sum of all x[i][j] (i!=j) must equal 1
        for(int j=0; j<n; j++) {
            for(int i=0; i<n; i++) {
                if(i!=j) {
                    expr += x[i][j];
                }
            }
            model.add(expr == 1);
            expr.clear();
        }
        /* u[i] is a dummy variable, if the solution has no subtour, the model can find the variables
        u[i] satisfy the constraint: u[i]-u[j]+n*x[i][j] <= n-1 with all i, j >=1 (i!=j). For easy of
        visualization, u[i] now represents the order of visits of the nodes. If
        the solution has subtours it is not possible. */

        IloNumVarArray u(env, n);
        for(int i=1; i<n; i++) {
            u[i] = IloNumVar(env, 1, n-1, ILOINT);
        }

        for(int i=1; i<n; i++) {
            for(int j=1; j<n; j++) {
                if(i!=j) {
                    expr = u[i]-u[j]+n*x[i][j];
                    model.add(expr <= n-1);
                    expr.clear();
                }
            }
        }

        expr.end();

        /*------SOLVE----------------*/

        IloCplex cplex(model);

        cplex.exportModel("TSP-MTZ_ulysses22.lp");
        cplex.solve();

        double sumCost = cplex.getObjValue();
        cout<< "The minimum cost = " << sumCost << "\n";
        cout<< "The path is: "<< "\n";
        // suppose we start from place 2
        int start_vertex = 2;
        cout<< "2";
        int i = 2;
        do{
            for(int j=0; j<n; j++) {
                if(i!=j) {
                    if(cplex.getValue(x[i][j])==1) {
                        cout<< "->" << j;
                        i = j;
                        break;
                    }
                }
            }
        } while(i!=start_vertex);
        cout<< "\n";

    }


    catch (IloException& e){
        cerr << "Conver exception caught: " << e << endl; // No solution exists
    }
    catch (...) {
        cerr << "Unknown exception caught" << endl;
    }

    env.end();

    return 0;
}
