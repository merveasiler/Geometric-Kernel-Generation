#include "sdlp.h"
#include "BaseGeoOpUtils.h"
#include "Mesh.h"

using namespace sdlp;

double* sdlpMain(double extremeDirection[3], vector<HalfSpace>& halfSpaceSet) {

    int d = 3;
    int m = halfSpaceSet.size();
    Eigen::VectorXd x(d);    // decision variables
    Eigen::VectorXd c(d);    // objective coefficients
    Eigen::MatrixXd A(m, d); // constraint matrix
    Eigen::VectorXd b(m);    // constraint bound

    //c << 0.0, 0.0, 1.0;   // default
    c << extremeDirection[0], extremeDirection[1], extremeDirection[2];

    for (int i = 0; i < halfSpaceSet.size(); i++) {
        HalfSpace hp = halfSpaceSet[i];
        A.row(i) << hp.ABCD[0], hp.ABCD[1], hp.ABCD[2];
        b(i) = -hp.ABCD[3];
    }

    double minobj = linprog(c, A, b, x);

    if (minobj == numeric_limits<double>::infinity()) {
        //cout << "INFEASIBLE" << endl;
        return NULL;
    }
    if (minobj == -numeric_limits<double>::infinity()) {
        //cout << "UNBOUNDED" << endl;
        return NULL;
    }

    double* kernel_point = new double[d];
    for (int i = 0; i < d; i++)
        kernel_point[i] = x[i];
    return kernel_point;

}

double* sdlpMain(Mesh hostMesh, double extremeDirection[3]) {

    vector<HalfSpace> halfSpaceSet;
    computeHalfSpacesFromTriangles(hostMesh.getAllTris(), hostMesh.getAllVerts(), halfSpaceSet);
    double* kernel_point = sdlpMain(extremeDirection, halfSpaceSet);
    return kernel_point;
}
