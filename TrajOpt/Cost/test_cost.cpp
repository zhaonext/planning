#include <iostream>

#include "cost.h"
int main() {
    ReferenceBezierCurve<REFERENCE_LINE_ORDER> bezierCurve;
    std::vector<std::vector<double>> controlPoints = {{0.0, 0.0}, {0.2, 0.2}, {0.8, 0.2}, {1.0, 0.0}};
    bezierCurve.setControlPoints(controlPoints);
    State X = {0.0, 0.0, 0.0, 0, 0.2, 0};

    auto cost = Cost(bezierCurve);
    auto result = cost.calInitState(X);

    result.print_state();
    return 0;
}