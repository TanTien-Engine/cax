#pragma once

#include "Geometry.h"
#include "Constraint.h"
#include "../thirdparty/PlaneGCS/GCS.h"

#include <memory>

namespace gs { class Shape2D; class Point2D; class Line2D; }

namespace gcsgen
{

class Scene
{
public:
	Scene();
	~Scene();

	void AddConstraint(const Constraint& cons);

	int Solve();

	void Clear();

private:
	void ClearConstraints();

	void ResetSolver();

	// geometries
	int AddGeometry(const std::shared_ptr<gs::Shape2D>& geo);
	int AddPoint(const std::shared_ptr<gs::Point2D>& pt);
	int AddLine(const std::shared_ptr<gs::Line2D>& line);

	// basic constraints
	int AddDistanceCons(int point1, PointPos pos1, int point2, PointPos pos2, double* value);
	int AddDistanceCons(int line, double* value);

	// derived constraints
	int AddHorizontalCons(int geo1, PointPos pos1, int geo2, PointPos pos2);
	int AddHorizontalCons(int line);
	int AddVerticalCons(int geo1, PointPos pos1, int geo2, PointPos pos2);
	int AddVerticalCons(int line);

	void BeforeSolve();
	void AfterSolve();

private:
	std::shared_ptr<GCS::System> m_gcs = nullptr;

	std::vector<double*> m_parameters;

	std::map<std::shared_ptr<gs::Shape2D>, int> m_geo_id;
	std::vector<Geometry> m_geos;

	GCS::Algorithm m_default_solver = GCS::DogLeg;
	GCS::Algorithm m_default_solver_redundant = GCS::DogLeg;

	std::vector<GCS::Point> m_points;
	std::vector<GCS::Line>  m_lines;

	int m_constraints_counter = 0;

}; // Scene

}