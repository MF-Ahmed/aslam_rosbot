#include <ceres/ceres.h>
#include <iostream>
#include <fstream>
#include <sstream>

struct EdgeSE2
{
    int id1;
    int id2;
    double x, y, theta;
};

struct VertexSE2
{
    int id;
    double x, y, theta;
};

class CostFunctionSE2
{
public:
    CostFunctionSE2(const EdgeSE2& edge)
        : edge_(edge)
    {
    }

    template<typename T>
    bool operator()(const T* const pose1, const T* const pose2, T* residual) const
    {
        T p1[2] = {pose1[0], pose1[1]};
        T p2[2] = {pose2[0], pose2[1]};
        T rotation = pose1[2];

        T c = ceres::cos(rotation);
        T s = ceres::sin(rotation);

        T x2 = c * p2[0] - s * p2[1];
        T y2 = s * p2[0] + c * p2[1];

        residual[0] = p1[0] + x2 - T(edge_.x);
        residual[1] = p1[1] + y2 - T(edge_.y);

        return true;
    }

private:
    const EdgeSE2& edge_;
};

int main(int argc, char** argv)
{
    if (argc < 2)
    {
        std::cout << "Usage: ./optimize_g2o <path_to_g2o_file>" << std::endl;
        return 1;
    }

    std::string g2oFilePath = argv[1];

    // Read the g2o file and populate vertices and edges
    std::ifstream inputFile(g2oFilePath);
    std::string line;
    std::vector<VertexSE2> vertices;
    std::vector<EdgeSE2> edges;

    while (std::getline(inputFile, line))
    {
        std::istringstream iss(line);
        std::string elementType;
        iss >> elementType;

        if (elementType == "VERTEX_SE2")
        {
            VertexSE2 vertex;
            iss >> vertex.id >> vertex.x >> vertex.y >> vertex.theta;
            vertices.push_back(vertex);
        }
        else if (elementType == "EDGE_SE2")
        {
            EdgeSE2 edge;
            iss >> edge.id1 >> edge.id2 >> edge.x >> edge.y >> edge.theta;
            edges.push_back(edge);
        }
    }

    // Create the Ceres problem
    ceres::Problem problem;

    // Add variables (poses)
    for (const auto& vertex : vertices)
    {
        double* pose = new double[3];
        pose[0] = vertex.x;
        pose[1] = vertex.y;
        pose[2] = vertex.theta;
        problem.AddParameterBlock(pose, 3);
    }

    // Add cost functions (constraints)
    for (const auto& edge : edges)
    {
        ceres::CostFunction* costFunction = new ceres::AutoDiffCostFunction<CostFunctionSE2, 2, 3, 3>(
            new CostFunctionSE2(edge)
        );

        problem.AddResidualBlock(costFunction, nullptr,
            problem.GetParameterBlockMutable(optimizedNodes[edge.id1]),
            problem.GetParameterBlockMutable(optimizedNodes[edge.id2])
        );
        optimizedEdges.push_back(edge); // Store optimized edge
    }

    // Configure and run the solver
    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_QR;
    options.minimizer_progress_to_stdout = true;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    // Print the optimization summary
    std::cout << summary.FullReport() << std::endl;

    // Retrieve the optimized values
    for (const auto& vertex : vertices)
    {
        const double* optimizedPose = problem.GetParameterBlock(vertex.id);
        std::cout << "Optimized pose for vertex " << vertex.id << ": "
                  << optimizedPose[0] << ", "
                  << optimizedPose[1] << ", "
                  << optimizedPose[2] << std::endl;
    }

    return 0;
}
