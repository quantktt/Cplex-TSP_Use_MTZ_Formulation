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
    //Create data (Random)
        srand(time(NULL));
                //suppose there are 20 place in the data
        int n=20;
        vector<double> xPos(n), yPos(n);
        int randXPos, randYPos;
        for(int i=0; i<n; i++) {
            //suppose coordinates between 0 and 1000
            randXPos = rand()%1000;
            randYPos = rand()%1000;
            xPos[i] = randXPos;
            yPos[i] = randYPos;
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
        /* u[i] denodes the order in which point i is accessed, u[i] = n denotes that place 0 must be
           accessed last. This ensures a valid cycle and a valid solution */
        //for all i,j >0 and i!=j: u[i]-u[j]+(n-1)*x[i][j] <= n-2
        IloNumVarArray u(env, n);
        for(int i=1; i<n; i++) {
            u[i] = IloNumVar(env, 0, IloInfinity, ILOFLOAT);
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
